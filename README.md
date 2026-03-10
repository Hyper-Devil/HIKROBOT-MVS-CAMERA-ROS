# HIKROBOT-MVS-CAMERA-ROS
The ros driver package of Hikvision Industrial Camera SDK. Support configuration parameters, the parameters have been optimized, and the photos have been transcoded to rgb format.
Please install mvs, https://blog.csdn.net/weixin_41965898/article/details/116801491

驱动会自动扫描 YAML 中所有满足完整标定字段的相机配置键（不限于 `camera_left/camera_right`），并按配置数量启动相机。
每个启用相机必须配置 `ip`，驱动将进行严格 IP 匹配：只要设备数量不一致或 IP 未全部精确匹配，节点将直接退出。

话题命名按配置键动态生成：
- `/hikrobot_camera/<camera_name>/image_raw` (+ `camera_info`)

# Install
```
mkdir -p ~/ws_hikrobot_camera/src
git clone https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS.git ~/ws_hikrobot_camera/src/hikrobot_camera
cd ~/ws_hikrobot_camera
catkin_make
```
# launch run
```
source ./devel/setup.bash 
roslaunch hikrobot_camera hikrobot_camera.launch
roslaunch hikrobot_camera hikrobot_camera_left_only.launch
```

`hikrobot_camera_left_only.launch` 只是示例：它加载的 YAML 只有一个相机键，所以只会启动一个相机。
