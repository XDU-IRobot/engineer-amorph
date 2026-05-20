"""Launch amorph_client_gateway nodes."""
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    mqtt_gateway = LifecycleNode(
        package='amorph_client_gateway',
        executable='mqtt_gateway_node',
        name='mqtt_gateway',
        namespace='',
        parameters=[{
            'mqtt_host': '192.168.12.1',
            'mqtt_port': 3333,
            'poll_period_ms': 50,
            'camera_frame': 'vt_camera_frame',
            'joint_names': [
                'j1_l', 'j2_l', 'j3_l', 'j4_l', 'j5_l', 'j6_l', 'gripper_joint_l',
                'j1_r', 'j2_r', 'j3_r', 'j4_r', 'j5_r', 'j6_r', 'gripper_joint_r',
            ],
        }],
        output='screen',
    )

    # Configure node
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(mqtt_gateway),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Activate after configure succeeds (configuring → inactive)
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mqtt_gateway,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(mqtt_gateway),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        # -- Video receiver (standalone) ----------------------------------
        Node(
            package='amorph_client_gateway',
            executable='vt_receiver_node',
            name='udp_video_receiver',
            parameters=[{
                'udp_port': 3334,
                'frame_id': 'vt_camera_optical_frame',
                'poll_period_sec': 0.01,
                'camera_name': 'vt_camera',
                'camera_info_url': 'package://amorph_bringup/config/vt_camera_calibration.yaml',
                'topic_namespace': 'vt_camera',
                'max_fps': 0.0,
            }],
            output='screen',
        ),

        # -- MQTT Gateway (lifecycle) -------------------------------------
        mqtt_gateway,
        configure_event,
        activate_event,

        # -- Static transforms ---------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_vt_camera_tf',
            arguments=[
                '--x', '0.3', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'vt_camera_frame',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='optical_to_vt_camera_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.5707', '--pitch', '0', '--yaw', '-1.5707',
                '--frame-id', 'vt_camera_frame',
                '--child-frame-id', 'vt_camera_optical_frame',
            ],
        ),
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
        ),
    ])
