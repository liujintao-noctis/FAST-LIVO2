# FAST-LIVO2 Copilot 使用指南

## 构建命令

本项目是一个 ROS（catkin）包，需在 catkin 工作区中构建：

```bash
# 标准 Release 构建
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# Debug 构建
cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Debug
```

项目要求 C++17。构建时会自动检测 CPU 架构（x86-64 或 ARM/aarch64），并应用对应的 `-march=native` / `-mcpu=native` 编译优化。`MP_EN` 和 `MP_PROC_NUM` 会根据核心数自动设置（上限为 4），用于 OpenMP 并行化。

## 运行方式

```bash
# 使用指定传感器配置启动
roslaunch fast_livo mapping_avia.launch
rosbag play <数据集>.bag

# 其他可用的 launch 目标：
# mapping_avia_marslvig.launch
# mapping_hesaixt32_hilti22.launch
# mapping_ouster_ntu.launch
```

每个 launch 文件会从 `config/` 目录加载对应的 YAML 配置。新增传感器或数据集支持时，需同时创建 `config/<name>.yaml` 和 `launch/mapping_<name>.launch`。

## 系统架构

### 顺序 ESIKF：LIO → VIO 有序更新

核心算法结构是**顺序误差状态迭代卡尔曼滤波器（Sequential ESIKF）**，对同一时间戳执行两轮更新——先进行 LiDAR 测量更新（LIO），再进行视觉测量更新（VIO）。这一机制分散在多个文件中，不容易从单个文件中直接看出：

- `LidarMeasureGroup`（`include/common_lib.h`）持有 `EKF_STATE` 标志位（`WAIT` / `LIO` / `VIO` / `LO`），控制每次调用执行哪种更新。
- `LIVMapper::sync_packages()`（`src/LIVMapper.cpp:884`）负责分发该标志，将 LiDAR 与图像数据交错排列，使得同一传感器时间戳产生两个顺序滤波步骤。
- 运行模式（`ONLY_LO`、`ONLY_LIO`、`LIVO`）在启动时由 `img_en` 和 `lidar_en` 参数决定，存储在 `slam_mode_` 中。

### 状态向量

`StatesGroup`（`include/common_lib.h`）保存 19 维状态：
`rot_end`（SO(3)）| `pos_end` | `vel_end` | `bias_g` | `bias_a` | `gravity` | `inv_expo_time`

协方差矩阵为 `MD(19,19)`，通过 `INIT_COV` 初始化。

### 体素地图

`VoxelMapManager`（`include/voxel_map.h`、`src/voxel_map.cpp`）使用 `unordered_map`，以三维体素坐标的哈希值为键（`VOXELMAP_HASH_P = 116101`）。每个体素包含一棵由 `OctoTree` 节点构成的八叉树，对局部平面进行带不确定度的建模。LiDAR EKF 更新在此处构建点到平面的残差。

### 视觉子系统

`VIOManager`（`include/vio.h`、`src/vio.cpp`）管理：
- `feat_map` — 以体素位置为键的 `VisualPoint*` 映射（世界坐标系下的三维地图点）
- `warp_map` — 每个跟踪点的仿射变形 patch

处理流程：可见性查询 → 深度连续性过滤 → 可选 raycasting → 多层金字塔光度 EKF 更新。

参考 patch 的选择逻辑和法向收敛判断分别位于 `include/feature.h` 和 `include/visual_point.h`。

### IMU 处理

`ImuProcess`（`src/IMU_Processing.cpp`）负责 `StatesGroup` 的前向传播、协方差传播，以及在进入 LiDAR 更新前对点云进行反向运动畸变补偿。

## 关键约定

### 类型别名（始终使用这些别名，不要直接使用原始 Eigen 类型）

定义于 `include/utils/types.h`：

```cpp
V3D       // Eigen::Vector3d
M3D       // Eigen::Matrix3d
MD(r,c)   // Eigen::Matrix<double, r, c>
VD(n)     // Eigen::Matrix<double, n, 1>
PointCloudXYZI  // pcl::PointCloud<pcl::PointXYZINormal>
```

### 支持的 LiDAR 类型

`include/common_lib.h` 中的 `LID_TYPE` 枚举：`AVIA=1`、`VELO16=2`、`OUST64=3`、`L515=4`、`XT32=5`、`PANDAR128=6`、`ROBOSENSE=7`。整数值通过 YAML 中的 `preprocess/lidar_type` 字段设置。

### 相机模型

相机内参使用 `vikit` 库。支持鱼眼（fisheye）和针孔（pinhole）两种模型，配置文件分别独立存放（如 `config/camera_fisheye_HILTI22.yaml`、`config/camera_pinhole.yaml`）。

### 由 YAML 控制的功能开关

以下算法选项在编译时默认关闭，可通过配置文件在运行时开启：
- `vio/raycast_en` — 启用基于遮挡感知的 raycasting 可见性检测
- `vio/inverse_composition_en` — 切换为逆合成（inverse compositional）公式
- `vio/normal_en` — 启用基于法向量的视觉点深度估计
- `vio/exposure_estimate_en` — 在线光度曝光时间估计

### 日志与调试路径

`DEBUG_FILE_DIR(name)` 展开为 `<仓库根目录>/Log/<name>`。`Log/` 目录已加入 `.gitignore`。所有调试文件输出均应使用此宏。

### 坐标系约定

- `point_b` — LiDAR 机体坐标系
- `point_i` — IMU 机体坐标系
- `point_w` — 世界坐标系

外参：`extrinT`/`extrinR` 为 IMU 到 LiDAR 的外参；`Rcl`/`Pcl` 为 LiDAR 到相机的外参。

## 提交规范

遵循本仓库使用的前缀风格：

```
[Feat] 添加对 <传感器/数据集> 的支持。
[Fix] 修正 <描述>。
[Docs] 更新 <描述>。
[Update] 改进 <描述>。
```

PR 应注明受影响的传感器或数据集，列出修改过的配置/launch 文件；若算法行为有变更，需附上 RViz 截图或轨迹对比图。

## 不应提交的内容

`build/` 和 `Log/` 已加入 `.gitignore`，不要提交 rosbag 文件、网格输出或派生地图数据。新增 YAML 参数时，需在 YAML 文件中添加行内注释，并在 PR 描述中说明其用途。
