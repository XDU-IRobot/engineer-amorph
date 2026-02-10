import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_respawn_arg = DeclareLaunchArgument(
        'enable_respawn',
        default_value='false',
        description='Enable automatic respawn on failure (for production use)'
    )
    package = "amorph_transport"
    package_shared_path = get_package_share_directory(package)
    node = Node(
        package=package,
        executable="amorph_transport_node",
        name="amorph_transport",
        output="screen",
        parameters=[
            os.path.join(package_shared_path, "config", "settings.yaml"),
        ],
        respawn=LaunchConfiguration('enable_respawn'),
        respawn_delay=2.0
    )

    return LaunchDescription([enable_respawn_arg, node])
