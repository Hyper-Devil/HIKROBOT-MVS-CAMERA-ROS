# 相机标定参数转换说明

## XML标定文件到YAML配置文件的转换

### 原始XML标定文件结构分析

XML文件包含以下关键参数：
- `size`: 图像尺寸 (2048 x 1536)
- `M1`: 左相机内参矩阵 (3x3)
- `D1`: 左相机畸变系数 (1x5)
- `M2`: 右相机内参矩阵 (3x3)
- `D2`: 右相机畸变系数 (1x5)
- `R`: 旋转矩阵 (3x3) - 右相机相对于左相机的旋转
- `T`: 平移向量 (3x1) - 右相机相对于左相机的平移

### 转换对应关系

#### 左相机 (camera_left)
- `image_width`: 2048 (从XML的size标签)
- `image_height`: 1536 (从XML的size标签)
- `camera_matrix`: M1矩阵数据
- `distortion_coefficients`: D1矩阵数据
- `rectification_matrix`: R矩阵数据 (用于双目立体校正)
- `projection_matrix`: 基于M1矩阵构建，Tx=0

#### 右相机 (camera_right)
- `image_width`: 2048 (从XML的size标签)
- `image_height`: 1536 (从XML的size标签)
- `camera_matrix`: M2矩阵数据
- `distortion_coefficients`: D2矩阵数据
- `rectification_matrix`: R矩阵的转置 (用于双目立体校正)
- `projection_matrix`: 基于M2矩阵构建，Tx根据基线距离计算

### 关键参数说明

#### 1. 相机内参矩阵 (K)
```
[fx  0  cx]
[0  fy  cy]
[0   0   1]
```
- fx, fy: 焦距参数
- cx, cy: 主点坐标

#### 2. 畸变系数 (D)
```
[k1, k2, p1, p2, k3]
```
- k1, k2, k3: 径向畸变系数
- p1, p2: 切向畸变系数

#### 3. 投影矩阵 (P)
```
[fx' 0  cx' Tx]
[0  fy' cy' Ty]
[0   0   1   0]
```
- 对于右相机，Tx = -fx * baseline_x
- baseline_x从T向量的第一个分量计算：-427.842mm

#### 4. 双目基线计算
从T向量可以得到：
- X方向基线：-427.842mm
- Y方向基线：-0.864mm  
- Z方向基线：1.499mm

右相机投影矩阵的Tx值计算：
```
Tx = -fx * baseline_x = -1224.955 * (-427.842/1000) = 523734.4
```

### 配置文件使用

1. **单相机模式**：使用 `camera_info_left_only.yaml`
2. **双相机模式**：使用 `camera_info.yaml`

### 验证方法

可以通过以下方式验证标定参数的正确性：
1. 检查内参矩阵的合理性（焦距、主点位置）
2. 验证畸变系数的范围
3. 确认双目基线距离的合理性
4. 运行相机节点检查图像发布是否正常

### 注意事项

1. 图像尺寸必须与实际相机输出一致
2. 畸变系数的符号和顺序需要正确
3. 双目标定时，左右相机的rectification_matrix应该形成正确的立体校正关系
4. 投影矩阵的Tx值对于深度估计至关重要
