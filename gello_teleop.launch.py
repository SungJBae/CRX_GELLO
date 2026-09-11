"""Bring up the GELLO teleop chain.

The FANUC driver is launched separately so you can restart teleop without
cycling the controller manager:

    ros2 launch fanuc_forward_command fanuc_forward_command.launch.py \
        robot_model:=crx10ia_l use_mock:=true
    ros2 launch gello_crx gello_teleop.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = LaunchConfiguration("params_file")
    use_gripper = LaunchConfiguration("use_gripper")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        get_package_share_directory("gello_crx"),
                        "config",
                        "gello_crx.yaml",
                    ]
                ),
                description="Parameter file for all three nodes.",
            ),
            DeclareLaunchArgument(
                "use_gripper",
                default_value="true",
                description="Launch the Robotiq 2F-140 bridge.",
            ),
            Node(
                package="gello_crx",
                executable="gello_leader",
                name="gello_leader",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="gello_crx",
                executable="gello_teleop",
                name="gello_teleop",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="gello_crx",
                executable="gello_gripper",
                name="gello_gripper",
                output="screen",
                parameters=[params],
                condition=IfCondition(use_gripper),
            ),
        ]
    )
