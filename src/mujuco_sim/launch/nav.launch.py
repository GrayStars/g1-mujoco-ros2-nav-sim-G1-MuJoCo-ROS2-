import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


WS_SRC = os.path.expanduser("~/g1_ws/src")


def generate_launch_description():
    network = LaunchConfiguration("network")
    domain = LaunchConfiguration("domain")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    fast_lio_config_path = LaunchConfiguration("fast_lio_config_path")
    fast_lio_config_file = LaunchConfiguration("fast_lio_config_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    fast_lio_delay = LaunchConfiguration("fast_lio_delay")
    nav2_delay = LaunchConfiguration("nav2_delay")

    package_share = FindPackageShare("mujuco_sim")
    g1_nav_launch = PathJoinSubstitution(
        [package_share, "launch", "g1_nav_sim.launch.py"]
    )
    default_params_file = PathJoinSubstitution(
        [package_share, "config", "g1_nav2_params.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "map.rviz"]
    )

    cyclone_uri = f"file://{WS_SRC}/mujuco_sim/config/cyclonedds_lo.xml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("network", default_value="lo"),
            DeclareLaunchArgument("domain", default_value="1"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(WS_SRC, "maps", "vln_navigation_room.yaml"),
            ),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument(
                "fast_lio_config_path",
                default_value=os.path.join(WS_SRC, "FAST_LIO_ROS2", "config"),
            ),
            DeclareLaunchArgument("fast_lio_config_file", default_value="mid360.yaml"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("fast_lio_delay", default_value="8.0"),
            DeclareLaunchArgument("nav2_delay", default_value="12.0"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", domain),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            SetEnvironmentVariable("CYCLONEDDS_URI", cyclone_uri),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(g1_nav_launch),
                launch_arguments={
                    "network": network,
                    "domain": domain,
                    "input": "cmd_vel",
                    "g1_ctrl_terminal": "true",
                    "publish_robot_odom": "false",
                    "publish_sensor_tf_static": "true",
                }.items(),
            ),
            TimerAction(
                period=fast_lio_delay,
                actions=[
                    Node(
                        package="fast_lio",
                        executable="fastlio_mapping",
                        name="fastlio_mapping",
                        output="screen",
                        parameters=[
                            PathJoinSubstitution(
                                [fast_lio_config_path, fast_lio_config_file]
                            ),
                            {"use_sim_time": False},
                        ],
                    ),
                    Node(
                        package="mujuco_sim",
                        executable="fastlio_tf_bridge",
                        name="fastlio_tf_bridge",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=nav2_delay,
                actions=[
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        output="screen",
                        parameters=[params_file],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_localization",
                        output="screen",
                        parameters=[
                            {"use_sim_time": False},
                            {"autostart": True},
                            {"node_names": ["map_server"]},
                        ],
                    ),
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        name="controller_server",
                        output="screen",
                        parameters=[params_file],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_planner",
                        executable="planner_server",
                        name="planner_server",
                        output="screen",
                        parameters=[params_file],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_recoveries",
                        executable="recoveries_server",
                        name="recoveries_server",
                        output="screen",
                        parameters=[params_file],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_bt_navigator",
                        executable="bt_navigator",
                        name="bt_navigator",
                        output="screen",
                        parameters=[params_file],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_waypoint_follower",
                        executable="waypoint_follower",
                        name="waypoint_follower",
                        output="screen",
                        parameters=[params_file],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="screen",
                        parameters=[params_file],
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
            ),
        ]
    )
