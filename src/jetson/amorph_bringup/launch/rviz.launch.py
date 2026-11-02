from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = list()
    declared_arguments.append(
        DeclareLaunchArgument(
            "jsp_gui",
            default_value="false",
            description="Start jsp gui",
        )
    )
    jsp_gui = LaunchConfiguration("jsp_gui")

    moveit_config = (
        MoveItConfigsBuilder("amorph", package_name="amorph_v5_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/amorph.urdf.xacro")
        # .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    
    nodes = list()

    # RViz
    rviz_config_file = (
        get_package_share_directory("amorph_bringup") + "/config/mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )
    nodes.append(rviz_node)

    # jsp gui
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(jsp_gui),
    )
    nodes.append(joint_state_publisher_gui_node)

    launch = LaunchDescription(declared_arguments + nodes)
    return launch
