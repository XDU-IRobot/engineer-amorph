
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "fluorite", package_name="fluorite_arm_moveit_config").to_moveit_configs()
    return LaunchDescription(
        [
            Node(
                package='fluorite_behaviour_tree',
                executable='bt_executor',
                output='both',
                parameters=[moveit_config.robot_description,
                            moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics],),
        ],
    )
