"""Launch the two amorph_client_gateway composable nodes in one container."""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='gateway_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='amorph_client_gateway',
                    plugin='amorph::UdpVideoReceiverNode',
                    name='udp_video_receiver',
                    parameters=[{
                        'udp_port': 3334,
                        'frame_id': 'camera_optical_frame',
                        'poll_period_sec': 0.01,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='amorph_client_gateway',
                    plugin='amorph::MqttGatewayNode',
                    name='mqtt_gateway',
                    parameters=[{
                        'mqtt_host': '192.168.12.1',
                        'mqtt_port': 3333,
                        'client_id': '102',
                        'poll_period_ms': 50,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        ),
    ])
