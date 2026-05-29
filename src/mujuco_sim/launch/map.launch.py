import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


MODE = "sim"  # "sim": MuJoCo + keyboard + FAST-LIO; "real": real Mid360 topics + FAST-LIO.
USE_SLAM_TOOLBOX_2D = False  # True: 2D /scan SLAM map; False: FAST-LIO PCD map.
SIM_FAST_LIO_CONFIG_FILE = "mid360.yaml"
REAL_FAST_LIO_CONFIG_FILE = "mid360_real.yaml"
MAP_DELAY_SECONDS = "8.0"
SIM_DOMAIN_ID = "1"
REAL_DOMAIN_ID = "0"
WS_SRC = os.path.expanduser("~/yushu_ws/src")
SIM_CYCLONEDDS_URI = f"file://{WS_SRC}/mujuco_sim/config/cyclonedds_lo.xml"
REAL_CYCLONEDDS_URI = ""  # Empty means: keep the shell/Unitree ROS2 network setup.


def generate_launch_description():
    network = LaunchConfiguration("network")
    domain = LaunchConfiguration("domain")
    fast_lio_config_path = LaunchConfiguration("fast_lio_config_path")
    rviz = LaunchConfiguration("rviz")
    map_delay = LaunchConfiguration("map_delay")

    package_share = FindPackageShare("mujuco_sim")
    fast_lio_share = FindPackageShare("fast_lio")
    g1_nav_launch = PathJoinSubstitution(
        [package_share, "launch", "g1_nav_sim.launch.py"]
    )
    fast_lio_mapping_launch = PathJoinSubstitution(
        [fast_lio_share, "launch", "mapping.launch.py"]
    )
    slam_toolbox_params = PathJoinSubstitution(
        [package_share, "config", "g1_slam_toolbox.yaml"]
    )
    rviz_config = PathJoinSubstitution([package_share, "rviz", "map.rviz"])

    mode = MODE.lower()
    if mode not in ("sim", "real"):
        raise RuntimeError("MODE must be 'sim' or 'real' in map.launch.py")

    fast_lio_config_file = (
        SIM_FAST_LIO_CONFIG_FILE if mode == "sim" else REAL_FAST_LIO_CONFIG_FILE
    )
    default_domain = SIM_DOMAIN_ID if mode == "sim" else REAL_DOMAIN_ID
    cyclone_uri = SIM_CYCLONEDDS_URI if mode == "sim" else REAL_CYCLONEDDS_URI

    actions = [
        DeclareLaunchArgument("network", default_value="lo"),
        DeclareLaunchArgument("domain", default_value=default_domain),
        DeclareLaunchArgument(
            "fast_lio_config_path",
            default_value=os.path.join(WS_SRC, "FAST_LIO_ROS2", "config"),
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("map_delay", default_value=MAP_DELAY_SECONDS),
        SetEnvironmentVariable("ROS_DOMAIN_ID", domain),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
    ]
    if cyclone_uri:
        actions.append(SetEnvironmentVariable("CYCLONEDDS_URI", cyclone_uri))

    if mode == "sim":
        actions.extend(
            [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(g1_nav_launch),
                launch_arguments={
                    "network": network,
                    "domain": domain,
                    "input": "keyboard",
                    "g1_ctrl_terminal": "true",
                }.items(),
            ),
            ]
        )

    if USE_SLAM_TOOLBOX_2D:
        actions.append(
            TimerAction(
                period=map_delay,
                actions=[
                    Node(
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        output="screen",
                        parameters=[slam_toolbox_params],
                    ),
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        arguments=["-d", rviz_config],
                        condition=IfCondition(rviz),
                    ),
                ],
            )
        )
    else:
        actions.append(
            TimerAction(
                period=map_delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(fast_lio_mapping_launch),
                        launch_arguments={
                            "config_path": fast_lio_config_path,
                            "config_file": fast_lio_config_file,
                            "rviz": rviz,
                            "use_sim_time": "false",
                        }.items(),
                    ),
                ],
            )
        )

    return LaunchDescription(actions)
