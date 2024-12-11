import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('fluorite_chassis'), 'config')
    config_file_full_path = [os.path.join(config_dir, "settings.yaml")]
    
    return LaunchDescription(
        [
            Node(
                package="dr16_node",
                executable="dr16_node",
                name="dr16_node",
                parameters=[{"tty_device": "/dev/rm_remote"}],
                output="screen"),
            Node(
                package="fluorite_chassis",
                executable="fluorite_chassis",
                name="fluorite_chassis",
                parameters=config_file_full_path,
                output="screen"),
        ],
    )