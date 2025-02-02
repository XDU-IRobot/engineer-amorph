
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

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

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    imu_node = Node(
        package="witmotion_ros2_driver",
        executable="witmotion_ros2_driver",
        name="witmotion_ros2_driver",
        parameters=[{"serial_port": "/dev/ttyS4",
                     "baud_rate": 921600, }],
        output="screen"
    )
    remote_node = Node(
        package="sbus_bridge",
        executable="sbus_bridge_node",
        name="sbus_bridge_node",
        parameters=[{"port": "/dev/ttyS0",
                     "refresh_rate_hz": 5,
                             "rxMinValue": 353,
                             "rxMaxValue": 1695,
                             "outMinValue": -1000,
                             "outMaxValue": 1000,
                             "silentOnFailsafe": False,
                             "enableChannelNum": -1,
                             "enableChannelProportionalMin": 0.0,
                     "enableChannelProportionalMax": 1.0, }],
        output="screen"
    )
    chassis_control_node = Node(
        package="fluorite_control",
        executable="fluorite_control_chassis",
        name="fluorite_control_chassis",
        parameters=[os.path.join(get_package_share_directory(
                    'fluorite_control'), "config", "settings.yaml")],
        output="screen"
    )
    arm_control_node = Node(
        package="fluorite_control",
        executable="fluorite_control_arm",
        name="fluorite_control_arm",
        parameters=[os.path.join(get_package_share_directory(
            'fluorite_control'), "config", "settings.yaml")],
        output="screen"
    )
            

    nodes = [
        robot_state_publisher_node,
        imu_node,
        remote_node,
        chassis_control_node,
        arm_control_node,
    ]
    
    launch = LaunchDescription(declared_arguments + nodes)
    launch.add_entity(generate_move_group_launch(moveit_config))

    return launch