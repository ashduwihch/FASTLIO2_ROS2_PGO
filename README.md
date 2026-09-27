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

## 本版本修改

### 1. 修正 ROS 2 IMU 加速度单位

`sensor_msgs/msg/Imu` 的线加速度单位已经是 `m/s²`。原代码再次乘以 `10.0`，
会把重力和运动加速度放大十倍。当前版本移除了这次重复缩放，避免姿态和点云漂移。

### 2. 增加空点云保护

Gazebo 启动、暂停、场景切换或雷达没有有效命中时，可能产生空点云。原程序会将
空点云送入 PCL，特定情况下触发除零异常并以 `exit code -8` 退出。当前版本会直接
忽略空帧，有效点云仍按原流程处理。

### 3. 补充 Sophus 链接目标

在 `fastlio2/CMakeLists.txt` 中显式链接 `Sophus::Sophus`，解决当前环境下可能出现的
Sophus 链接问题。该修改不改变算法。

### 4. 优化 PGO 的 RViz 默认显示

默认关闭容易造成长期显示卡顿的累计 `world_cloud` 和路径显示，并增加里程计箭头。
需要查看累计世界点云时，可在 RViz 中手动启用 `/fastlio2/world_cloud`。

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
  ros-humble-pcl-conversions ros-humble-tf2-ros
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

也可以将该指令加入 `~/.bashrc`。

以后新开终端运行本项目时，需要先加载 ROS 2、Livox 驱动和本项目三个环境：

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/fastlio2_pgo_ws/install/setup.bash
```

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
source ~/fastlio2_pgo_ws/install/setup.bash
ros2 launch fastlio2 lio_launch.py
```

## 启动 FAST-LIO2 + PGO 回环

```bash
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

```bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/绝对路径/地图目录', save_patches: true}"
```

## 重定位

```bash
ros2 launch localizer localizer_launch.py
```

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

## 与 MID-360 Gazebo 插件配合

推荐启动顺序：

1. 启动 Gazebo、PX4 和 MID-360；
2. 确认 `/livox/lidar` 与 `/livox/imu` 正常发布；
3. 启动 `fastlio2 lio_launch.py`，或直接启动带回环的 `pgo pgo_launch.py`。

检查传感器话题：

```bash
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## 项目来源与致谢

- [`liangheming/FASTLIO2_ROS2`](https://github.com/liangheming/FASTLIO2_ROS2)
- [`hku-mars/FAST_LIO`](https://github.com/hku-mars/FAST_LIO)
- [`hku-mars/BALM`](https://github.com/hku-mars/BALM)
- [`hku-mars/HBA`](https://github.com/hku-mars/HBA)

本仓库保留原项目提交历史和各软件包中的许可证文件。修改与再发布时请继续遵守
对应的开源许可证。
