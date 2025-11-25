from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='referee_node',
                executable='referee_node',
                name='referee_node',
                output='screen',
                parameters=[{
                    "enable_normal": True,
                    "enable_fpv": True,
                    "new_vt": True,
                    "normal_tty_device": "/dev/ttyUSB0",
                    "fpv_tty_device": "/dev/ttyS1",
                }],
            )
        ]
    )
