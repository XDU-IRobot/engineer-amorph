from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    enable_respawn_arg = DeclareLaunchArgument(
        'enable_respawn',
        default_value='false',
        description='Enable automatic respawn on failure (for production use)'
    )
    moveit_config = (
        MoveItConfigsBuilder("amorph", package_name="amorph_arm_v2_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/amorph.urdf.xacro")
        # .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    nodes = list()
    
    # Composable nodes for static TF publishing and robot state publisher
    composable_node_container = ComposableNodeContainer(
        name="static_tf_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        respawn=LaunchConfiguration('enable_respawn'),
        respawn_delay=2.0,
        composable_node_descriptions=[
            # Static TF(world -> arm_base)
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf_world_to_base",
                parameters=[{
                    "child_frame_id": "arm_base",
                    "frame_id": "world",
                }],
            ),
            # Static TF(arm_base -> link_1)
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf_base_to_arm",
                parameters=[{
                    "child_frame_id": "l1_l",
                    "frame_id": "arm_base",
                }],
            ),
            # Static TF(arm_base -> camera_link)
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf_base_to_cam",
                parameters=[{
                    "child_frame_id": "camera_link",
                    "frame_id": "arm_base",
                    "translation.x": -0.5,  # 用easy_handeye做好手眼标定之后把结果填到这来
                    "translation.y": -0.1577,
                    "translation.z": 0.43,
                    "rotation.x": -0.004162,
                    "rotation.y": -0.023165,
                    "rotation.z": -0.002643,
                    "rotation.w": 0.999720,
                }],
            ),
            # Robot state publisher
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
        ],
        output="log",
    )
    nodes.append(composable_node_container)

    launch = LaunchDescription([enable_respawn_arg] + nodes)
    launch.add_entity(generate_move_group_launch(moveit_config))
    return launch
