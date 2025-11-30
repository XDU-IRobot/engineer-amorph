import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='marchtag_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory(
                    'amorph_bringup'), "config/apriltag.yaml")
            ],
            remappings=[
                ('/image_rect', '/camera/color/image_raw'),
                ('/camera_info', '/camera/color/camera_info')
            ]
        )
    ])
