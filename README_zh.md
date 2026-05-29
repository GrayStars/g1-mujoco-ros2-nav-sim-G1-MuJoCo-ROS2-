# G1 MuJoCo ROS2 导航联合仿真

这个工作空间把 Unitree G1 MuJoCo 仿真、MJLab 策略部署、ROS2 Navigation、2D SLAM，以及 Mid360 / FAST-LIO 建图流程整合到一起。目标是在 MuJoCo 里验证导航、感知和行走策略，再逐步迁移到真实 G1 机器人。

## 实现功能

- 使用 `unitree_mujoco` 启动 Unitree G1 MuJoCo 仿真。
- 使用 `unitree_rl_mjlab/deploy/robots/g1/g1_ctrl` 部署并运行 G1 行走策略。
- 支持 Nav2 输出 `/cmd_vel` 控制 G1 行走。
- MuJoCo 到 ROS2 的桥接节点发布：
  - `/scan`
  - `/livox/lidar`
  - `/imu/data`
  - `/odom`
  - `/tf`
- 在 G1 头部模拟 Livox Mid360 三维点云。
- 支持 `slam_toolbox` 进行 2D 建图。
- 支持 `FAST_LIO_ROS2` 进行 3D PCD 建图和定位实验。
- 支持 Nav2 读取保存好的 2D `.yaml/.pgm` 地图进行导航。

## 总体数据流

```text
Nav2 目标点
  -> Nav2 planner/controller
  -> /cmd_vel
  -> g1_ctrl --cmd_vel
  -> MJLab / Unitree 行走策略
  -> Unitree MuJoCo 机器人运动
  -> shared memory
  -> mujuco_sim bridge
  -> /scan /livox/lidar /imu/data /tf
  -> SLAM / FAST-LIO / Nav2 / RViz
```

## 目录结构

```text
src/
  mujuco_sim/              # ROS2 launch 文件和 MuJoCo-ROS 桥接节点
  unitree_mujoco/          # Unitree MuJoCo 仿真器，已加入导航共享内存和 Mid360 仿真
  unitree_rl_mjlab/        # MJLab G1 策略部署和 g1_ctrl
  FAST_LIO_ROS2/           # FAST-LIO2 ROS2 包
  Livox-SDK2/              # Livox SDK2 依赖
  livox_ros_driver2/       # Livox ROS2 驱动，主要用于实机
  unitree_sdk2/            # Unitree SDK2
  unitree_ros2/            # Unitree ROS2 示例和接口
  maps/                    # 2D 地图和保存的 PCD 地图
```

## 环境要求

当前主要在 Ubuntu 22.04 + ROS2 Humble 下测试。

安装 ROS2 导航相关依赖：

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-pcl-ros
```

MuJoCo、Unitree SDK2、Unitree MuJoCo、MJLab 的环境配置请参考 Unitree 官方文档。使用本工作空间之前，建议先确认下面两个基础流程能跑通：

```bash
# Unitree MuJoCo 能启动，并且能看到 G1 机器人。
cd ~/yushu_ws/src/unitree_mujoco/simulate/build
./unitree_mujoco

# MJLab / Unitree 策略控制器能控制机器人。
cd ~/yushu_ws/src/unitree_rl_mjlab/deploy/robots/g1/build
./g1_ctrl --network=lo --domain=1 --keyboard
```

## 编译

新电脑第一次拿到这个 workspace，或者拷贝整个工作空间后，建议先清空旧的编译产物再编译：

```bash
cd ~/yushu_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

后续日常开发一般可以直接增量编译；只有依赖、生成文件或包结构变化时，再清理 `build install log`。

如果遇到 `stand_go2` duplicate package 错误，请确认下面这个文件存在：

```text
src/unitree_mujoco/example/COLCON_IGNORE
```

## 启动 MuJoCo + G1 控制

键盘控制模式：

```bash
cd ~/yushu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mujuco_sim g1_nav_sim.launch.py input:=keyboard
```

Nav2 `/cmd_vel` 控制模式：

```bash
ros2 launch mujuco_sim g1_nav_sim.launch.py input:=cmd_vel
```

常用检查命令：

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /livox/lidar
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo base_link livox_frame
```

## 使用 slam_toolbox 进行 2D 建图

修改 `src/mujuco_sim/launch/map.launch.py` 顶部参数：

```python
MODE = "sim"
USE_SLAM_TOOLBOX_2D = True
```

启动建图：

```bash
cd ~/yushu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mujuco_sim map.launch.py
```

用键盘控制机器人走动建图，然后保存 2D 地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/yushu_ws/src/maps/new_2d_map
```

## 使用 FAST-LIO 保存 PCD 地图

修改 `src/mujuco_sim/launch/map.launch.py` 顶部参数：

```python
MODE = "sim"
USE_SLAM_TOOLBOX_2D = False
```

启动 FAST-LIO 建图：

```bash
cd ~/yushu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mujuco_sim map.launch.py
```

控制机器人在环境里走动。注意：不要依赖 `Ctrl+C` 保存 PCD，必须显式调用保存服务：

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

仿真 FAST-LIO 的 PCD 输出路径在下面文件里配置：

```text
src/FAST_LIO_ROS2/config/mid360.yaml
```

默认保存到：

```text
./src/maps/vln_fastlio_map.pcd
```

## 启动导航

确认 `src/mujuco_sim/launch/nav.launch.py` 中使用的是需要的 2D 地图。默认地图为：

```text
src/maps/vln_navigation_room.yaml
```

启动导航：

```bash
cd ~/yushu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mujuco_sim nav.launch.py
```

在 RViz 中发送 `2D Goal Pose`。Nav2 会发布 `/cmd_vel`，`g1_ctrl --cmd_vel` 接收后驱动 MuJoCo 中的 G1 行走。

## 实机部署说明

实机 G1 上的大致流程：

1. 按 Unitree 官方文档配置网络、SDK2 和 ROS2。
2. 启动真实 Mid360 驱动，确认存在 `/livox/lidar`。
3. 如果要用 `slam_toolbox` 建 2D 图，需要用 `pointcloud_to_laserscan` 把 Mid360 的 `PointCloud2` 转成 `/scan`。
4. 保存 `.yaml/.pgm` 地图后给 Nav2 使用。
5. 在真实机器人网络接口上运行 `g1_ctrl`，例如：

```bash
cd ~/yushu_ws/src/unitree_rl_mjlab/deploy/robots/g1/build
./g1_ctrl --network=enp5s0
```

建议先验证键盘控制和策略部署没问题，再切换到 `/cmd_vel` 模式接入 Nav2。

## Launch 文件说明

- `g1_nav_sim.launch.py`：启动 Unitree MuJoCo、`g1_ctrl` 和共享内存 ROS2 bridge。
- `map.launch.py`：启动建图流程；在文件顶部切换 2D `slam_toolbox` 或 FAST-LIO PCD 建图。
- `nav.launch.py`：启动仿真、FAST-LIO 定位桥接、Nav2 和 RViz。

## DDS 说明

仿真默认使用 `ROS_DOMAIN_ID=1`，并通过下面配置让 CycloneDDS 使用本机 loopback：

```text
src/mujuco_sim/config/cyclonedds_lo.xml
```

本仓库的 launch 文件会自动设置这些环境变量。
