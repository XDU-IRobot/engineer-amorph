import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fluorite", package_name="fluorite_arm_moveit_config").to_moveit_configs()
    
    return LaunchDescription(
        [
            Node(
                package="fluorite_control",
                executable="fluorite_control_test_ik",
                name="fluorite_control_test_ik",
                parameters=[os.path.join(get_package_share_directory(
                    'fluorite_control'), "config", "settings.yaml"), 
                            moveit_config.robot_description,
                            moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics],
                output="screen"),
        ],
    )
