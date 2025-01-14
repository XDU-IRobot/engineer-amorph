from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="witmotion_ros2_driver",
                executable="witmotion_ros2_driver",
                name="witmotion_ros2_driver",
                parameters=[{"serial_port": "/dev/ttyS4",
                            "baud_rate": 921600,}],
                output="screen"),
        ],
    )