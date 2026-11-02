from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    enable_respawn_arg = DeclareLaunchArgument(
        'enable_respawn',
        default_value='false',
        description='Enable automatic respawn on failure (for production use)'
    )
    joint_state_source_arg = DeclareLaunchArgument(
        'jsp_gui',
        default_value='false',
        description='Launch jsp gui'
    )
    moveit_config = (
        MoveItConfigsBuilder("amorph", package_name="amorph_v5_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/amorph.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    world_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        arguments=[
            "--frame-id", "world",
            "--child-frame-id", "camera_link",
        ],
        respawn=LaunchConfiguration('enable_respawn'),
        respawn_delay=2.0
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[moveit_config.robot_description],
        condition=IfCondition(
            EqualsSubstitution(
                LaunchConfiguration('jsp_gui'), 'true')),
    )

    nodes = [world_to_camera_tf, joint_state_publisher_gui]

    launch = LaunchDescription(
        [enable_respawn_arg, joint_state_source_arg] + nodes)
    launch.add_entity(generate_rsp_launch(moveit_config))
    return launch
