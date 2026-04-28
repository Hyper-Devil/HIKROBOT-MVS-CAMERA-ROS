# HIKROBOT-MVS-CAMERA-ROS

ROS 1 driver for Hikvision industrial cameras (GigE / USB3 Vision) via the MVS SDK.  
Supports single and multi-camera setups, hardware/software triggering, and on-the-fly stereo/mono rectification.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
  - [Camera Operation — camera.yaml](#camera-operation--camerayaml)
  - [Camera Calibration — camera\_info\_\*.yaml](#camera-calibration--camera_info_yaml)
- [Multi-Camera Setup](#multi-camera-setup)
- [Published Topics](#published-topics)
- [Rectification](#rectification)
- [Offline Rectification from a Bag](#offline-rectification-from-a-bag)
- [Calibration File Format](#calibration-file-format)
- [Known Limitations](#known-limitations)

---

## Prerequisites

| Dependency | Notes |
|---|---|
| ROS Noetic (Ubuntu 20.04) | Other distros may work but are untested. |
| Hikvision MVS SDK 2.x | GigE and USB3 Vision. [Install guide](https://blog.csdn.net/weixin_41965898/article/details/116801491) |
| OpenCV 4.x | Bundled with ROS Noetic. |

---

## Installation

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS.git hikrobot_camera
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Quick Start

```bash
# Single camera
roslaunch hikrobot_camera hikrobot_camera_left_only.launch

# Two cameras (edit config/camera_info_multi.yaml with real IPs first)
roslaunch hikrobot_camera hikrobot_camera_multi.launch
```

---

## Configuration

### Camera Operation — `camera.yaml`

Loaded by all launch files. Controls acquisition behaviour, shared across all cameras.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `TriggerMode` | int | `0` | `0` = free-run (continuous), `1` = external trigger. |
| `TriggerSource` | int | `0` | Trigger input: `0`=Line0 `1`=Line1 `2`=Line2 `3`=Line3 `4`=Counter0 `7`=Software `8`=FreqConverter. |
| `FrameRate` | float | `10.0` | Target frame rate (Hz) in free-run mode. Ignored when `TriggerMode=1`. |
| `FrameTimeoutMs` | int | `60` | Per-frame SDK timeout (ms). For 20 Hz external trigger use ≥ 50. |
| `SystemTime` | bool | `false` | `true` = use host PC clock for timestamps; `false` = use camera hardware timestamp converted to ROS time. |

### Camera Calibration — `camera_info_*.yaml`

Each top-level YAML key defines one camera. The key name becomes the camera's **logical name** used for topic naming and device matching.

**Required fields per camera:**

| Field | Description |
|---|---|
| `ip` | GigE camera IP address. Strict exact match — node exits if any configured IP is not found. |
| `image_width` / `image_height` | Sensor output resolution. Must match the camera's actual output. |
| `camera_matrix.data` | 3×3 intrinsic matrix K, row-major, 9 elements. |
| `distortion_coefficients.data` | `[k1, k2, p1, p2, k3]`, 5 elements. |
| `rectification_matrix.data` | 3×3 rectification rotation R, row-major, 9 elements. Use identity for mono. |
| `projection_matrix.data` | 3×4 projection matrix P, row-major, 12 elements. For stereo right camera `P[3]` = −fx × baseline\_m. |
| `distortion_model` | Always `"plumb_bob"`. |

**Optional fields:**

| Field | Default | Description |
|---|---|---|
| `binning_x` / `binning_y` | `1` | Hardware binning factor. Used when computing the ROI crop offset. |
| `roi.x_offset`, `roi.y_offset`, `roi.width`, `roi.height` | `0` | Crop window in the **full rectified** image (pixels). |
| `roi.do_rectify` | `false` | Set `true` to enable online rectification and publish `/rect/image_rect`. |

---

## Multi-Camera Setup

1. Edit (or copy) `config/camera_info_multi.yaml`. Add one top-level key per camera:

```yaml
camera_front:
  ip: "192.168.1.10"
  image_width: 2048
  image_height: 1536
  camera_matrix:
    rows: 3
    cols: 3
    data: [fx, 0, cx,  0, fy, cy,  0, 0, 1]
  distortion_coefficients:
    rows: 1
    cols: 5
    data: [k1, k2, p1, p2, k3]
  rectification_matrix:
    rows: 3
    cols: 3
    data: [1,0,0, 0,1,0, 0,0,1]
  projection_matrix:
    rows: 3
    cols: 4
    data: [fx, 0, cx, 0,  0, fy, cy, 0,  0, 0, 1, 0]
  distortion_model: "plumb_bob"
  binning_x: 1
  binning_y: 1
  roi:
    x_offset: 0
    y_offset: 0
    width: 2048
    height: 1536
    do_rectify: false

camera_rear:
  ip: "192.168.1.11"
  # ... same fields ...
```

2. **The driver requires the number of YAML entries to equal the number of physically connected cameras exactly.** Any mismatch causes an immediate exit with a descriptive error.

3. For more than two cameras simply add more keys — no code changes are needed. The driver auto-discovers all keys that contain the required calibration fields.

---

## Published Topics

For each configured camera key `<name>`:

| Topic | Type | Condition |
|---|---|---|
| `/hikrobot_camera/<name>/image_raw` | `sensor_msgs/Image` (RGB8) | Always. |
| `/hikrobot_camera/<name>/camera_info` | `sensor_msgs/CameraInfo` | Always. |
| `/hikrobot_camera/<name>/rect/image_rect` | `sensor_msgs/Image` (RGB8) | Only when `roi.do_rectify: true`. |
| `/hikrobot_camera/<name>/rect/camera_info` | `sensor_msgs/CameraInfo` | Only when `roi.do_rectify: true`. |

---

## Rectification

When `roi.do_rectify: true`, the driver performs per-frame online rectification:

1. **Map initialisation** (once at startup):

   ```
   cv::initUndistortRectifyMap(K, D, R, P[:3×3], image_size, CV_32FC1, map_x, map_y)
   ```

   If the 3×3 sub-block of P has zero diagonal entries, K is used instead.

2. **Per frame** — remap then crop:

   ```
   rectified_full = cv::remap(raw, map_x, map_y, INTER_LINEAR)
   rect_cropped   = rectified_full[crop_y:crop_y+crop_h, crop_x:crop_x+crop_w]
   ```

3. **Published `rect/camera_info`** has D zeroed, R set to identity, and the principal point shifted by the crop offset: `cx' = P[0,2] − crop_x`, `cy' = P[1,2] − crop_y`. The stereo baseline terms `P[3]` (Tx) and `P[7]` (Ty) are preserved unchanged.

The `roi` rectangle is in the **full rectified** coordinate frame. Example — 2048×1536 sensor, centred 1536×1152 crop:

```yaml
roi:
  x_offset: 256
  y_offset: 192
  width: 1536
  height: 1152
  do_rectify: true
```

---

## Offline Rectification from a Bag

如果录制了 `image_raw` + `camera_info` 话题，可用离线矫正节点回放时补发矫正结果。该节点复用 `include/rect_utils.hpp` 中的 `buildRectifyArtifacts`，与在线驱动共享完全相同的实现（`initUndistortRectifyMap` 参数、`INTER_LINEAR` remap、ROI 裁剪及 `camera_info` 调整逻辑**完全一致**）。

**终端 1 — 播放 bag：**

```bash
rosbag play your_recording.bag
```

**终端 2 — 启动矫正节点：**

```bash
roslaunch hikrobot_camera rect_from_bag.launch
```

或让 launch 文件内部播放 bag：

```bash
roslaunch hikrobot_camera rect_from_bag.launch bag:=/path/to/your_recording.bag
```

节点自动扫描 `<prefix>/*/image_raw` + `<prefix>/*/camera_info` 话题对，为每对话题独立建立矫正管线，发布 `<prefix>/*/rect/image_rect` 与 `<prefix>/*/rect/camera_info`。多相机时各路回调并行执行（`ros::AsyncSpinner`）。

**节点源文件：** [`src/rect_from_bag.cpp`](src/rect_from_bag.cpp)  
**共享矫正逻辑：** [`include/rect_utils.hpp`](include/rect_utils.hpp)

**Launch 参数：**

| 参数 | 默认值 | 说明 |
|---|---|---|
| `bag` | `""` | bag 文件路径；留空则在外部单独播放。 |
| `bag_args` | `""` | 转发给 `rosbag play` 的额外参数，例如 `--clock -r 0.5`。 |
| `topic_prefix` | `/hikrobot_camera` | 只处理该前缀下的话题对；话题被 remap 时修改此项。 |
| `scan_timeout` | `10.0` | 等待话题出现的超时秒数，超时后节点报错退出。 |
| `queue_size` | `10` | 订阅/发布队列深度。 |

---

## Calibration File Format

See [`calibration_conversion_guide.md`](calibration_conversion_guide.md) for a step-by-step guide to converting OpenCV XML stereo calibration files into the YAML format used by this package.

Key rules:

- **Mono camera**: `rectification_matrix` = identity; `projection_matrix` column 4 all zeros.
- **Stereo left camera**: `projection_matrix` Tx (`P[3]`) = 0.
- **Stereo right camera**: `projection_matrix` Tx = −fx × baseline\_metres  
  (e.g. −1224.955 × 0.427842 ≈ −523734).

---

## Known Limitations

- **GigE only for IP matching.** USB cameras are enumerated by the SDK but cannot be matched by IP. Extend `findDeviceIndexByIp` in `include/hikrobot_camera.hpp` to add USB support.
- **Strict device count.** The YAML entry count must equal the number of physically connected cameras at startup. Intentional — prevents silent channel mismatches.
- **No runtime reconfiguration.** Trigger mode, frame rate, and calibration parameters are loaded once. Restart the node after editing YAML.
- **Rectification requires `roi.do_rectify: true`.** The `/rect/image_rect` topic is always advertised but frames are only published when this flag is set and the ROI dimensions are valid (non-zero width and height).
- **All cameras share one trigger configuration.** `TriggerMode`, `TriggerSource`, and `FrameRate` in `camera.yaml` are applied identically to every connected camera.
