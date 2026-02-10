from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    locator_node = LifecycleNode(
        package="energy_unit_locator",
        executable="locator_v2",
        name="energy_unit_locator_node",
        namespace="",
        parameters=[{
            "model_path": "",
            "confidence_threshold": 0.5,
            "erosion_kernel_size": 5,
            "erosion_iterations": 1,
            "publish_masked_pointcloud": True,
            "publish_annotated_image": False,
            "smoothing_alpha": 0.3,
        }],
        output="screen"
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
        locator_node,
        configure_event,
        activate_event,
    ])
