# 仓库指南

## 项目结构与模块组织
`src/` 包含运行时主流程：`main.cpp` 启动 ROS 节点，`LIVMapper.cpp` 负责建图主逻辑，`vio.cpp`、`voxel_map.cpp`、`preprocess.cpp` 和 `IMU_Processing.cpp` 等文件实现核心状态估计模块。公共头文件位于 `include/`，共享工具代码在 `include/utils/`，Livox 消息定义位于 `include/livox_ros_driver/`。运行配置放在 `config/*.yaml`，不同数据集或传感器的启动文件在 `launch/`，RViz 预设在 `rviz_cfg/`。辅助脚本位于 `scripts/`，论文和图片资料分别在 `doc/`、`Supplementary/` 和 `pics/`。

## 论文与实现概述
`doc/FAST-LIVO2Fast, Direct LiDAR-Inertial-Visual.pdf` 是本仓库的算法主参考。可按下表快速建立“论文章节 -> 源码模块”对应关系：

| 论文模块 | 主要代码 | 作用 |
| --- | --- | --- |
| 系统总览与主调度 | `src/main.cpp`, `src/LIVMapper.cpp` | ROS 节点入口、参数读取、主循环、LIO/VIO 切换 |
| 顺序 ESIKF 与状态定义 | `include/common_lib.h`, `src/LIVMapper.cpp` | 定义 19 维状态；`sync_packages()` 将同一时刻数据拆成先 LiDAR 后图像的顺序更新 |
| IMU 传播与点云去畸变 | `src/IMU_Processing.cpp` | 前向传播、协方差传播、反向补偿点云运动畸变 |
| 统一体素地图与局部地图 | `include/voxel_map.h`, `src/voxel_map.cpp` | Hash + Octree 体素结构、平面建模、不确定度估计、地图滑窗 |
| LiDAR 测量模型 | `src/voxel_map.cpp` | 构造点到平面残差并执行 LiDAR 迭代 Kalman 更新 |
| 视觉点选择与可见性检索 | `src/vio.cpp` | 可见体素查询、深度连续性剔除、按需 raycasting |
| 视觉直接法更新 | `src/vio.cpp` | patch 仿射变换、光度残差、多层金字塔视觉 EKF |
| 视觉地图点与参考 patch 更新 | `include/feature.h`, `include/visual_point.h`, `src/vio.cpp` | 新视觉点挂接 patch、参考 patch 选择、法向收敛判断 |

贡献算法代码时，优先先查论文对应节，再进入上述模块。需注意，开源实现中部分增强项由 `config/*.yaml` 控制或做了简化，例如 `raycast_en`、`inverse_composition_en` 以及较保守的法向更新策略。

## 构建、测试与开发命令
请在 catkin 工作区中构建此包：

```bash
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

运行默认示例：

```bash
roslaunch fast_livo mapping_avia.launch
rosbag play <dataset>.bag
```

调试时可使用 `catkin_make -DCMAKE_BUILD_TYPE=Debug`。项目已在 `CMakeLists.txt` 中启用 C++17，并根据 CPU 架构设置了对应的 Release 优化参数。`scripts/mesh.py` 和 `scripts/colmap_output.sh` 这类脚本属于离线工具，不属于主构建流程。

## 编码风格与命名约定
遵循现有 C++ 风格：使用 2 空格缩进，函数和类的大括号另起一行；修改已有文件时，尽量保持与周边代码一致的头文件组织方式。命名规则以仓库现状为准：类名使用 PascalCase，如 `LIVMapper`；源文件名称通常采用描述性命名，如 `voxel_map.cpp`、`IMU_Processing.cpp`；ROS 话题名和配置键保持 snake_case。仓库中没有本地格式化配置文件，因此提交时应控制改动范围，并优先保持风格一致。

## 测试说明
当前仓库没有独立的 `tests/` 目录，也没有现成的 CI 测试套件。验证修改时，至少应重新执行 `catkin_make`，启动相关 `launch/*.launch` 文件，并回放具有代表性的 rosbag。若新增自动化测试，优先使用 `package.xml` 中已经声明的 `rostest`，并采用与模块对应的测试命名方式，例如 `test_preprocess.test`。

## 提交与 Pull Request 规范
近期提交记录采用简短前缀风格，例如 `[Feat] add support for HILTI22 dataset.` 和 `[Fix] correct quaternion component order.`。请沿用这一模式：使用 `[Feat]`、`[Fix]`、`[Docs]` 或 `[Update]`，后接简洁的祈使式摘要。PR 应说明受影响的传感器或数据集，列出改动过的配置文件或启动文件，并附上验证依据。若修改涉及算法行为或可视化结果，应附带 RViz 截图、轨迹图，或简短的 bag 回放说明。

## 数据与配置管理
不要提交 `build/` 或 `Log/` 下生成的输出，这两个目录已在 `.gitignore` 中忽略。大型 rosbag、网格文件和派生地图数据应保存在仓库之外；新增 YAML 参数时，应同时在对应配置文件和 PR 描述中说明其用途。
