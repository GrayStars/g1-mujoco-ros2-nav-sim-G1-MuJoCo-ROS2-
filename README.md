# G1 MuJoCo ROS2 Navigation Simulation

This workspace connects Unitree G1 MuJoCo simulation, MJLab policy deployment, ROS2 navigation, 2D SLAM, and simulated Mid360 / FAST-LIO mapping into one reproducible workflow. The goal is to test navigation and locomotion policies in MuJoCo before moving the same idea toward a real G1 robot.

## What This Provides

- Unitree G1 MuJoCo simulation using `unitree_mujoco`.
- G1 locomotion control using `unitree_rl_mjlab/deploy/robots/g1/g1_ctrl`.
- `/cmd_vel` control mode for Nav2.
- MuJoCo-to-ROS2 bridge publishing `/scan`, `/livox/lidar`, `/imu/data`, `/odom`, and TF.
- Simulated Livox Mid360 point cloud mounted on the G1 head.
- 2D mapping with `slam_toolbox`.
- 3D mapping / localization experiments with `FAST_LIO_ROS2`.
- Nav2 navigation with saved 2D `.yaml/.pgm` maps.

## Main Data Flow

```text
Nav2 goal
  -> Nav2 planner/controller
  -> /cmd_vel
  -> g1_ctrl --cmd_vel
  -> MJLab / Unitree walking policy
  -> Unitree MuJoCo robot motion
  -> shared memory
  -> mujuco_sim bridge
  -> /scan /livox/lidar /imu/data /tf
  -> SLAM / FAST-LIO / Nav2 / RViz
```

## Repository Layout

```text
src/
  mujuco_sim/              # ROS2 launch files and MuJoCo-ROS bridge nodes
  unitree_mujoco/          # Unitree MuJoCo simulator, modified for nav shared memory and Mid360
  unitree_rl_mjlab/        # MJLab G1 policy deployment and g1_ctrl
  FAST_LIO_ROS2/           # FAST-LIO2 ROS2 package
  Livox-SDK2/              # Livox SDK2 dependency
  livox_ros_driver2/       # Livox ROS2 driver, mainly for real robot use
  unitree_sdk2/            # Unitree SDK2
  unitree_ros2/            # Unitree ROS2 examples/interfaces
  maps/                    # 2D maps and saved PCD maps
```

## Prerequisites

Tested on Ubuntu 22.04 + ROS2 Humble.

Install ROS2 navigation dependencies:

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-pcl-ros
```

Install and configure MuJoCo, Unitree SDK2, Unitree MuJoCo, and MJLab dependencies following the official Unitree documentation. Before using this workspace, make sure these two standalone checks work:

```bash
# Unitree MuJoCo simulator can start and show the G1 robot.
cd ~/yushu_ws/src/unitree_mujoco/simulate/build
./unitree_mujoco

# MJLab / Unitree policy controller can control the robot.
cd ~/yushu_ws/src/unitree_rl_mjlab/deploy/robots/g1/build
./g1_ctrl --network=lo --domain=1 --keyboard
```

## Build

On a new machine, or after copying this workspace, do a clean build first:

```bash
cd ~/yushu_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

For later incremental development, `rm -rf build install log` is usually not required unless dependencies, generated files, or package layouts changed.

If `stand_go2` duplicate package errors appear, keep this file in place:

```text
src/unitree_mujoco/example/COLCON_IGNORE
```

## Run MuJoCo + G1 Control

Keyboard control:

```bash
cd ~/yushu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mujuco_sim g1_nav_sim.launch.py input:=keyboard
```

Nav2 `/cmd_vel` control:

```bash
ros2 launch mujuco_sim g1_nav_sim.launch.py input:=cmd_vel
```

Useful checks:

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /livox/lidar
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo base_link livox_frame
```

## 2D Mapping With slam_toolbox

Edit the top of `src/mujuco_sim/launch/map.launch.py`:

```python
MODE = "sim"
USE_SLAM_TOOLBOX_2D = True
```

Run mapping:

```bash
cd ~/yushu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mujuco_sim map.launch.py
```

Drive the robot with the keyboard controller, then save the 2D map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/yushu_ws/src/maps/new_2d_map
```

## FAST-LIO PCD Mapping

Edit the top of `src/mujuco_sim/launch/map.launch.py`:

```python
MODE = "sim"
USE_SLAM_TOOLBOX_2D = False
```

Run FAST-LIO mapping:

```bash
cd ~/yushu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mujuco_sim map.launch.py
```

Drive the robot around the scene. Do not rely on `Ctrl+C` to save the PCD. Save explicitly with:

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

The simulated FAST-LIO PCD output path is configured in:

```text
src/FAST_LIO_ROS2/config/mid360.yaml
```

By default it saves to:

```text
./src/maps/vln_fastlio_map.pcd
```

## Navigation

Make sure `src/mujuco_sim/launch/nav.launch.py` points to the desired 2D map. The default is:

```text
src/maps/vln_navigation_room.yaml
```

Launch navigation:

```bash
cd ~/yushu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mujuco_sim nav.launch.py
```

In RViz, send a `2D Goal Pose`. Nav2 publishes `/cmd_vel`, and `g1_ctrl --cmd_vel` executes the walking policy in MuJoCo.

## Real Robot Notes

For a real G1 robot:

1. Configure Unitree network, SDK2, and ROS2 according to Unitree official docs.
2. Start the real Mid360 driver and confirm `/livox/lidar`.
3. For 2D mapping with `slam_toolbox`, convert Mid360 `PointCloud2` to `/scan` using `pointcloud_to_laserscan`.
4. Save the resulting `.yaml/.pgm` map and use it with Nav2.
5. Run `g1_ctrl` on the real robot network interface, for example:

```bash
cd ~/yushu_ws/src/unitree_rl_mjlab/deploy/robots/g1/build
./g1_ctrl --network=enp5s0
```

After keyboard control is validated on the real robot, switch the controller to `/cmd_vel` mode for Nav2 integration.

## Launch File Summary

- `g1_nav_sim.launch.py`: starts Unitree MuJoCo, `g1_ctrl`, and the shared-memory ROS2 bridge.
- `map.launch.py`: starts simulation mapping; switch between 2D `slam_toolbox` and FAST-LIO PCD mapping at the top of the file.
- `nav.launch.py`: starts simulation, FAST-LIO localization bridge, Nav2, and RViz.

## DDS Notes

Simulation defaults to `ROS_DOMAIN_ID=1` and CycloneDDS loopback config:

```text
src/mujuco_sim/config/cyclonedds_lo.xml
```

The launch files set these automatically for local simulation.
