import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package = "amorph_transport"
    package_shared_path = get_package_share_directory(package)
    node = Node(
        package=package,
        executable="amorph_transport_node",
        output="screen",
        parameters=[
            os.path.join(package_shared_path, "config", "settings.yaml"),
        ],
    )

    return LaunchDescription([node])
