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

    can_dev_arg = DeclareLaunchArgument(
        'can_dev',
        default_value='can0',
        description='SocketCAN device name'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='/joint_states publish rate (Hz)'
    )

    node = Node(
        package='amorph_transport',
        executable='amorph_transport_node',
        name='amorph_transport',
        output='screen',
        parameters=[{
            'can_dev': LaunchConfiguration('can_dev'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        respawn=LaunchConfiguration('enable_respawn'),
        respawn_delay=2.0,
    )

    return LaunchDescription([
        enable_respawn_arg,
        can_dev_arg,
        publish_rate_arg,
        node,
    ])

