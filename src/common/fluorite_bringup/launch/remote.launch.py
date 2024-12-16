from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="dr16_node",
                executable="dr16_node",
                name="dr16_node",
                parameters=[{"tty_device": "/dev/rm_remote"}],
                output="screen"),
        ],
    )