from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="d435_align",
            executable="d435_align",
            name="d435_align",
            output="screen",
            parameters=[{
                "world_frame": "world",
                "d435_frame": "camera_link",
                "arm_frame": "base_link",
            }]
        )
    ])