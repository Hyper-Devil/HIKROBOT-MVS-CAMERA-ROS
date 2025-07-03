# HIKROBOT MVS Camera ROS Driver - 修改说明

## 主要修改

### 1. 参数加载方式改变
- **原来**: 通过硬编码路径读取YAML文件
- **现在**: 通过ROS参数服务器读取参数

### 2. 相机可用性检测
- 自动检测`camera_left`和`camera_right`参数是否存在
- 只启动有参数配置的相机
- 如果只有左相机参数，右相机不会被启动

### 3. 使用方法

#### 双相机配置
```bash
# 使用原始的配置文件（包含左右相机）
roslaunch hikrobot_camera hikrobot_camera.launch
```

#### 单相机配置
```bash
# 使用只有左相机的配置文件
roslaunch hikrobot_camera hikrobot_camera_left_only.launch
```

### 4. 配置文件结构

#### camera.yaml
```yaml
TriggerMode: 0
TriggerSource: 0
SysteamTime: false
```

#### camera_info.yaml (双相机)
```yaml
camera_left:
  image_width: 1280
  image_height: 1024
  camera_matrix:
    data: [...]
  # ... 其他参数

camera_right:
  image_width: 1280
  image_height: 1024
  camera_matrix:
    data: [...]
  # ... 其他参数
```

#### camera_info_left_only.yaml (单相机)
```yaml
camera_left:
  image_width: 1280
  image_height: 1024
  camera_matrix:
    data: [...]
  # ... 其他参数
```

### 5. 关键功能

- **自动检测**: 程序会自动检测哪些相机参数存在
- **灵活启动**: 只启动有参数配置的相机
- **错误处理**: 如果没有任何相机参数，程序会报错并退出
- **日志输出**: 清晰的日志显示哪些相机被启动

### 6. 修改的主要函数

- `checkCameraParamsExist()`: 检查相机参数是否存在
- `loadCameraInfoFromParams()`: 从ROS参数服务器加载相机参数
- `Camera()`: 构造函数中添加相机可用性检测
- `WorkThread()`: 工作线程中添加相机可用性检查

这些修改使得驱动程序更加灵活，可以根据配置自动适应单相机或双相机的使用场景。
