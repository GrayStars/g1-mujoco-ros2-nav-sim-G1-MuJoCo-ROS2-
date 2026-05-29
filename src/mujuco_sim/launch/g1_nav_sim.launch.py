import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


WS_SRC = os.path.expanduser("~/g1_ws/src")
MUJOCO_ROOT = os.path.join(WS_SRC, "unitree_mujoco")
MJLAB_ROOT = os.path.join(WS_SRC, "unitree_rl_mjlab")
MUJUCO_SIM_ROOT = os.path.join(WS_SRC, "mujuco_sim")


def generate_launch_description():
    unitree_mujoco_bin = LaunchConfiguration("unitree_mujoco_bin")
    unitree_mujoco_cwd = LaunchConfiguration("unitree_mujoco_cwd")
    g1_ctrl_bin = LaunchConfiguration("g1_ctrl_bin")
    g1_ctrl_cwd = LaunchConfiguration("g1_ctrl_cwd")
    network = LaunchConfiguration("network")
    domain = LaunchConfiguration("domain")
    input_mode = LaunchConfiguration("input")
    start_mujoco = LaunchConfiguration("start_mujoco")
    start_g1_ctrl = LaunchConfiguration("start_g1_ctrl")
    start_nav_bridge = LaunchConfiguration("start_nav_bridge")
    publish_robot_odom = LaunchConfiguration("publish_robot_odom")
    publish_sensor_tf_static = LaunchConfiguration("publish_sensor_tf_static")
    mujoco_terminal = LaunchConfiguration("mujoco_terminal")
    g1_ctrl_terminal = LaunchConfiguration("g1_ctrl_terminal")

    cyclone_uri = f"file://{MUJUCO_SIM_ROOT}/config/cyclonedds_lo.xml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "unitree_mujoco_bin",
                default_value=os.path.join(
                    MUJOCO_ROOT, "simulate", "build", "unitree_mujoco"
                ),
            ),
            DeclareLaunchArgument(
                "unitree_mujoco_cwd",
                default_value=os.path.join(MUJOCO_ROOT, "simulate", "build"),
            ),
            DeclareLaunchArgument(
                "g1_ctrl_bin",
                default_value=os.path.join(
                    MJLAB_ROOT, "deploy", "robots", "g1", "build", "g1_ctrl"
                ),
            ),
            DeclareLaunchArgument(
                "g1_ctrl_cwd",
                default_value=os.path.join(
                    MJLAB_ROOT, "deploy", "robots", "g1", "build"
                ),
            ),
            DeclareLaunchArgument("network", default_value="lo"),
            DeclareLaunchArgument("domain", default_value="1"),
            DeclareLaunchArgument("input", default_value="keyboard"),
            DeclareLaunchArgument("start_mujoco", default_value="true"),
            DeclareLaunchArgument("start_g1_ctrl", default_value="true"),
            DeclareLaunchArgument("start_nav_bridge", default_value="true"),
            DeclareLaunchArgument("publish_robot_odom", default_value="true"),
            DeclareLaunchArgument("publish_sensor_tf_static", default_value="false"),
            DeclareLaunchArgument("mujoco_terminal", default_value="false"),
            DeclareLaunchArgument("g1_ctrl_terminal", default_value="true"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", domain),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            SetEnvironmentVariable("CYCLONEDDS_URI", cyclone_uri),
            ExecuteProcess(
                cmd=[unitree_mujoco_bin],
                cwd=unitree_mujoco_cwd,
                additional_env={
                    "ROS_DOMAIN_ID": domain,
                    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                    "CYCLONEDDS_URI": cyclone_uri,
                },
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", start_mujoco, "' == 'true' and '", mujoco_terminal, "' != 'true'"]
                    )
                ),
            ),
            ExecuteProcess(
                cmd=[
                    "gnome-terminal",
                    "--title",
                    "unitree_mujoco",
                    "--",
                    "bash",
                    "-lc",
                    PythonExpression(
                        [
                            "'export ROS_DOMAIN_ID=",
                            domain,
                            "; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
                            "; export CYCLONEDDS_URI=",
                            cyclone_uri,
                            "; cd ",
                            unitree_mujoco_cwd,
                            " && ",
                            unitree_mujoco_bin,
                            "; exec bash'",
                        ]
                    ),
                ],
                additional_env={
                    "ROS_DOMAIN_ID": domain,
                    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                    "CYCLONEDDS_URI": cyclone_uri,
                },
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", start_mujoco, "' == 'true' and '", mujoco_terminal, "' == 'true'"]
                    )
                ),
            ),
            ExecuteProcess(
                cmd=[
                    g1_ctrl_bin,
                    "--network",
                    network,
                    "--domain",
                    domain,
                    PythonExpression(
                        [
                            "'--keyboard' if '",
                            input_mode,
                            "' == 'keyboard' else '--cmd_vel' if '",
                            input_mode,
                            "' == 'cmd_vel' else ''",
                        ]
                    ),
                ],
                cwd=g1_ctrl_cwd,
                additional_env={
                    "ROS_DOMAIN_ID": domain,
                    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                    "CYCLONEDDS_URI": cyclone_uri,
                },
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", start_g1_ctrl, "' == 'true' and '", g1_ctrl_terminal, "' != 'true'"]
                    )
                ),
            ),
            ExecuteProcess(
                cmd=[
                    "gnome-terminal",
                    "--title",
                    "g1_ctrl",
                    "--",
                    "bash",
                    "-lc",
                    PythonExpression(
                        [
                            "'export ROS_DOMAIN_ID=",
                            domain,
                            "; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
                            "; export CYCLONEDDS_URI=",
                            cyclone_uri,
                            "; cd ",
                            g1_ctrl_cwd,
                            " && ",
                            g1_ctrl_bin,
                            " --network ",
                            network,
                            " --domain ",
                            domain,
                            " ' + ('--keyboard' if '",
                            input_mode,
                            "' == 'keyboard' else '--cmd_vel' if '",
                            input_mode,
                            "' == 'cmd_vel' else '') + '; exec bash'",
                        ]
                    ),
                ],
                additional_env={
                    "ROS_DOMAIN_ID": domain,
                    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                    "CYCLONEDDS_URI": cyclone_uri,
                },
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", start_g1_ctrl, "' == 'true' and '", g1_ctrl_terminal, "' == 'true'"]
                    )
                ),
            ),
            Node(
                package="mujuco_sim",
                executable="unitree_mujoco_shm_bridge",
                name="unitree_mujoco_shm_bridge",
                output="screen",
                parameters=[
                    {"publish_robot_odom": publish_robot_odom},
                    {"publish_sensor_tf_static": publish_sensor_tf_static},
                ],
                condition=IfCondition(start_nav_bridge),
            ),
        ]
    )
