import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    package = "amorph_host"
    package_shared_path = get_package_share_directory(package)
    node = Node(
        package=package,
        executable="amorph_host_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            os.path.join(package_shared_path, "config", "pick_place_task.yaml"),
        ],
    )

    return LaunchDescription([node])
