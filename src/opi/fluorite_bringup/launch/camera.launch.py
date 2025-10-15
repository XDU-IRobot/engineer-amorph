
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
            package='rmos_cam',
            executable='daheng_camera',
            output='both',
            parameters=[os.path.join(get_package_share_directory(
                    'rmos_cam'), "config", "daheng_cam_settings.yaml")],),
        ],
    )