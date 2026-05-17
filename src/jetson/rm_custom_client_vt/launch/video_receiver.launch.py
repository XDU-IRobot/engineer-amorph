"""Launch the UDP H.265 video receiver node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_custom_client_vt',
            executable='udp_video_receiver',
            name='udp_video_receiver',
            output='screen',
            parameters=[{
                'udp_port': 3334,
                'frame_id': 'camera_optical_frame',
                'poll_period_sec': 0.01,
            }],
        ),
    ])
