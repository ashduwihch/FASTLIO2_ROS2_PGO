# FASTLIO2 ROS 2 PGO

本项目基于 [`liangheming/FASTLIO2_ROS2`](https://github.com/liangheming/FASTLIO2_ROS2)
进行整理与仿真适配，支持 **Ubuntu 22.04 + ROS 2 Humble**，包含：

- FAST-LIO2 激光惯性里程计；
- 基于位置先验、ICP 与 GTSAM 的 PGO 回环优化；
- 基于粗到细 ICP 的重定位；
- 基于 BALM / HBA 的一致性地图优化。

当前版本针对 Gazebo Harmonic MID-360 仿真补充了 IMU 单位修正、空点云保护、
Sophus 链接兼容和 RViz 显示优化，可直接配合
[`ashduwihch/mid360_gazebo_harmonic`](https://github.com/ashduwihch/mid360_gazebo_harmonic)
使用。

## 版本记录

| 版本 | 日期 | 主要变化 |
| --- | --- | --- |
| `v0.2.0` | 2026-10-13 | PGO 支持退出时自动保存优化地图；将重定位更新为 ROS 2 移植版 FAST_LIO_LOCALIZATION，支持 RViz `2D Pose Estimate` 初值、粗到细 Open3D ICP、匹配分数输出和持续 scan-to-map 校正。 |
| `v0.1.0` | 2026-09-27 | 完成 Ubuntu 22.04、ROS 2 Humble 与 Gazebo Harmonic MID-360 仿真适配。 |

## 与 MID-360 Gazebo 插件配合

推荐启动顺序：

以下启动步骤需要持续运行，请分别在不同终端中执行。

1. 使用 `mid360_posix_sitl.launch.py` 启动 Gazebo、PX4 和 MID-360：

   ```bash
   cd ~/PX4-Autopilot/launch
   ros2 launch ./mid360_posix_sitl.launch.py
   ```

2. 确认 `/livox/lidar` 与 `/livox/imu` 正常发布；
3. 启动 FAST-LIO2。仅运行里程计和建图：

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/livox_ws/install/setup.bash
   source ~/fastlio2_pgo_ws/install/setup.bash
   ros2 launch fastlio2 lio_launch.py
   ```

   或直接启动 FAST-LIO2、PGO 回环和 RViz：

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/livox_ws/install/setup.bash
   source ~/fastlio2_pgo_ws/install/setup.bash
   ros2 launch pgo pgo_launch.py
   ```

   两种方式选择一种即可，不要同时启动。

检查传感器话题：

```bash
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## 本版本修改

### 1. 修正 ROS 2 IMU 加速度单位

`sensor_msgs/msg/Imu` 的线加速度单位已经是 `m/s²`。原代码再次乘以 `10.0`，
会把重力和运动加速度放大十倍。当前版本移除了这次重复缩放，避免姿态和点云漂移。

### 2. 增加空点云保护

Gazebo 启动、暂停、场景切换或雷达没有有效命中时，可能产生空点云。原程序会将
空点云送入 PCL，特定情况下触发除零异常并以 `exit code -8` 退出。当前版本会直接
忽略空帧，有效点云仍按原流程处理。

### 3. 补充 Sophus 链接目标

在 FAST-LIO2 与 HBA 的构建配置中显式链接 `Sophus::Sophus`，解决已经找到
Sophus、但编译时仍提示 `sophus/so3.hpp` 不存在的问题。该修改不改变算法。

### 4. 优化 PGO 的 RViz 默认显示

默认关闭容易造成长期显示卡顿的累计 `world_cloud` 和路径显示，并增加里程计箭头。
需要查看累计世界点云时，可在 RViz 中手动启用 `/fastlio2/world_cloud`。

### 5. 增加 PGO 自动保存地图

支持在正常使用 `Ctrl+C` 结束 `pgo_node` 时，将优化后的地图和位姿保存到配置目录；
原有 `/pgo/save_maps` 手动保存服务仍然保留。

### 6. 更新重定位模块

将重定位更新为 `HViktorTsoi/FAST_LIO_LOCALIZATION` 的 ROS 2 移植实现，保留其
Open3D scan-to-map ICP、匹配分数判断和坐标变换融合思路，并适配当前 FAST-LIO2、
MID-360 话题与 RViz `2D Pose Estimate` 交互。

详细差异见 [`本版本修改说明.txt`](本版本修改说明.txt)。

## 环境与依赖

- Ubuntu 22.04
- ROS 2 Humble
- PCL
- Eigen
- Sophus
- GTSAM
- Livox-SDK2
- `livox_ros_driver2`

本项目使用 Livox `CustomMsg` 接收 MID-360 点云，因此编译前必须确保
`livox_ros_driver2` 已安装并能被当前 ROS 2 环境找到。

下面按照从一台已安装 ROS 2 Humble 的 Ubuntu 22.04 电脑开始，依次安装全部关键依赖。

## 1. 安装基础编译依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions \
  libpcl-dev libeigen3-dev libyaml-cpp-dev libboost-all-dev libtbb-dev \
  libopen3d-dev \
  ros-humble-rclcpp ros-humble-rviz2 ros-humble-std-msgs \
  ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs \
  ros-humble-visualization-msgs ros-humble-message-filters \
  ros-humble-pcl-conversions ros-humble-tf2 ros-humble-tf2-ros \
  ros-humble-rosidl-default-generators
```

## 2. 编译并安装 Livox-SDK2

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ~/Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

`Livox-SDK2` 是 Livox 的底层通信库。即使使用 Gazebo 仿真，后续编译
`livox_ros_driver2` 时仍需要它。

## 3. 编译 livox_ros_driver2

必须将驱动放在 ROS 2 工作空间的 `src` 目录中：

```bash
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/livox_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
```

编译完成后加载驱动环境：

```bash
source ~/livox_ws/install/setup.bash
```

对于本项目，`livox_ros_driver2` 不仅可连接真实 MID-360，也提供
`livox_ros_driver2/msg/CustomMsg` 消息接口；Gazebo MID-360 插件发布的点云正是该类型。

## 4. 编译并安装 Sophus

按照原项目使用的版本安装：

```bash
cd ~
git clone https://github.com/strasdat/Sophus.git
cd ~/Sophus
git checkout 1.22.10
mkdir -p build
cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j$(nproc)
sudo make install
```

## 5. 安装 GTSAM

PGO 和 HBA 需要 GTSAM。Ubuntu 22.04 可使用 GTSAM 官方提供的 PPA：

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository ppa:borglab/gtsam-release-4.2
sudo apt update
sudo apt install -y libgtsam-dev libgtsam-unstable-dev
```

如果选择从源码编译 GTSAM 4.2，请在 CMake 配置时加入：

```bash
-DGTSAM_USE_SYSTEM_EIGEN=ON
```

否则 GTSAM 可能使用自带的 Eigen 3.3.7，而 ROS 2 / PCL 使用系统 Eigen 3.4，
编译 HBA 时会因 Eigen 版本不一致而失败。

## 6. 创建工作空间并下载本项目

```bash
mkdir -p ~/fastlio2_pgo_ws/src
cd ~/fastlio2_pgo_ws/src
git clone https://github.com/ashduwihch/FASTLIO2_ROS2_PGO.git
```

## 7. 编译本项目

```bash
cd ~/fastlio2_pgo_ws
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

编译完成后加载环境：

```bash
source ~/fastlio2_pgo_ws/install/setup.bash
```

以后新开终端运行本项目时，需要先加载 ROS 2、Livox 驱动和本项目三个环境：

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/fastlio2_pgo_ws/install/setup.bash
```

编译全部完成后，可执行下面的指令将它们加入 `~/.bashrc`。这组指令只需执行一次：

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/livox_ws/install/setup.bash' >> ~/.bashrc
echo 'source ~/fastlio2_pgo_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

以后新开终端时会自动加载，不需要再次手动执行 `source`。

## 输入话题

默认配置文件：

```text
fastlio2/config/lio.yaml
```

默认输入：

| 数据 | 话题 | 消息类型 |
|---|---|---|
| MID-360 点云 | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` |
| IMU | `/livox/imu` | `sensor_msgs/msg/Imu` |

如果实际设备或仿真使用其他话题，请修改 `lio.yaml` 中的 `lidar_topic` 和
`imu_topic`。

## 启动 FAST-LIO2

仅运行激光惯性里程计和 RViz：

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/fastlio2_pgo_ws/install/setup.bash
ros2 launch fastlio2 lio_launch.py
```

## 启动 FAST-LIO2 + PGO 回环

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/fastlio2_pgo_ws/install/setup.bash
ros2 launch pgo pgo_launch.py
```

`pgo_launch.py` 已经同时启动 FAST-LIO2、PGO 和 RViz，使用回环时不要再单独启动
`lio_launch.py`，否则会出现重复节点和话题冲突。

PGO 配置位于：

```text
pgo/config/pgo.yaml
```

PGO 会持续检测可能的历史重访位置并进行位姿图优化，不是只执行一次回环。

## 保存地图

当前配置默认在正常使用 `Ctrl+C` 结束 `pgo_node` 时自动保存，配置位于
`pgo/config/pgo.yaml`：

```yaml
map_save:
  save_on_shutdown: true
  save_path: /home/lin/fastlio2_pgo_ws/src/FASTLIO2_ROS2_PGO/pgo/maps
  save_patches: false
```

程序会自动创建保存目录，并生成 `map.pcd` 和 `poses.txt`。当
`save_patches: true` 时还会生成 `patches/`，供 HBA 使用。
如需关闭退出自动保存，将 `save_on_shutdown` 改为 `false`。

运行过程中仍可随时手动保存：

```bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/绝对路径/地图目录', save_patches: true}"
```

## 重定位

```bash
ros2 launch localizer localizer_launch.py
```

启动后会自动加载 `localizer/config/localizer.yaml` 中 `map_path` 指向的先验地图。
在 RViz 中点击 **2D Pose Estimate**，然后在地图上按住鼠标拖出表示大致朝向的箭头，
程序会接收 `/initialpose` 并自动执行粗到细两阶段 ICP。箭头提供 `x、y、yaw`，
当箭头消息没有给出高度且 `override_initial_z: true` 时，`z` 使用配置文件中的
`initial_z`；`roll` 和 `pitch` 保持 RViz 消息给出的值（2D 工具通常为 0）。

默认配置为：

```yaml
map_path: /home/lin/fastlio2_pgo_ws/src/FASTLIO2_ROS2_PGO/pgo/maps/map.pcd
localization_frequency: 0.5
localization_threshold: 0.95
override_initial_z: true
initial_z: 0.5
```

终端的 `fitness score` 高于 `localization_threshold` 表示匹配成功，之后会按照
`localization_frequency` 持续使用先验地图校正。原来的服务接口仍然保留，可作为备用方式手动
指定地图和六自由度初值：

设置重定位初始值：

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/绝对路径/map.pcd', x: 0.0, y: 0.0, z: 0.0, \
  yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

## HBA 一致性地图优化

```bash
ros2 launch hba hba_launch.py
```

调用优化服务：

```bash
ros2 service call /hba/refine_map interface/srv/RefineMap \
  "{maps_path: '/绝对路径/地图目录'}"
```

使用 HBA 时，保存地图需设置 `save_patches: true`。

## 项目来源与致谢

- [`liangheming/FASTLIO2_ROS2`](https://github.com/liangheming/FASTLIO2_ROS2)
- [`hku-mars/FAST_LIO`](https://github.com/hku-mars/FAST_LIO)
- [`hku-mars/BALM`](https://github.com/hku-mars/BALM)
- [`hku-mars/HBA`](https://github.com/hku-mars/HBA)
- [`HViktorTsoi/FAST_LIO_LOCALIZATION`](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)

本仓库保留原项目提交历史和各软件包中的许可证文件。修改与再发布时请继续遵守
对应的开源许可证。
