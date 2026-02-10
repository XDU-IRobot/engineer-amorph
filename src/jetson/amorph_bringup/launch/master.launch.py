import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


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

    package = "amorph_master"
    package_shared_path = get_package_share_directory(package)
    node = Node(
        package=package,
        executable="amorph_master_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            os.path.join(package_shared_path, "config", "pick_place_task.yaml"),
            os.path.join(package_shared_path, "config", "pick_energy_unit_task.yaml")
                ,{"octomap_resolution": 0.02}
        ],
        respawn=LaunchConfiguration('enable_respawn'),
        respawn_delay=2.0
    )

    return LaunchDescription([enable_respawn_arg, node])
