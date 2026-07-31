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

    locator_node = Node(
        package="amorph_vision",
        executable="eu_locator_v3_node",
        name="eu_locator_node",
        namespace="",
        output="screen",
        respawn=LaunchConfiguration('enable_respawn'),
        respawn_delay=2.0
    )

    return LaunchDescription([
        enable_respawn_arg,
        locator_node,
    ])
