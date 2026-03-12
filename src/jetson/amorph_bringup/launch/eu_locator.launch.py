from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    enable_respawn_arg = DeclareLaunchArgument(
        'enable_respawn',
        default_value='false',
        description='Enable automatic respawn on failure (for production use)'
    )
    
    locator_node = LifecycleNode(
        package="amorph_vision",
        executable="locator_v2",
        name="energy_unit_locator_node",
        namespace="",
        parameters=[{
            "model_path": "",
            "confidence_threshold": 0.5,
            "erosion_kernel_size": 5,
            "erosion_iterations": 1,
            "publish_masked_pointcloud": False,
            "publish_annotated_image": True,
            "smoothing_alpha": 0.3,
        }],
        output="screen",
        respawn=LaunchConfiguration('enable_respawn'),
        respawn_delay=2.0
    )
    
    # 配置节点
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(locator_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    # 配置成功后激活节点
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=locator_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(locator_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    
    return LaunchDescription([
        enable_respawn_arg,
        locator_node,
        configure_event,
        activate_event,
    ])
