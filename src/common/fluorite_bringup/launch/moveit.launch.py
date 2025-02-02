
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    declared_arguments = list()

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="fluorite_arm_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="fluorite.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "jsp",
            default_value="false",
            description="Start joint state publisher gui",
        )
    )
    jsp = LaunchConfiguration("jsp")


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(LaunchConfiguration("description_package")),
                 "urdf", LaunchConfiguration("description_file")]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    moveit_config = MoveItConfigsBuilder("fluorite", package_name="fluorite_arm_moveit_config").to_moveit_configs()

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(jsp),
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
    ]
    
    launch = LaunchDescription(declared_arguments + nodes)
    launch.add_entity(generate_move_group_launch(moveit_config))

    return launch