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
    package = "amorph_master"
    package_shared_path = get_package_share_directory(package)

    moveit_config = (
        MoveItConfigsBuilder(
            "amorph", package_name="amorph_v5_moveit_config")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"])
        .robot_description(file_path="config/amorph.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # Pilz 的 LIN/CIRC 需要 robot_description_planning.cartesian_limits
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        # MoveItPy 的 pipeline / plan_request 参数（顶层合并）
        .moveit_cpp(file_path=os.path.join(
            package_shared_path, "config", "pipelines.yaml"))
        .to_moveit_configs()
    )

    node = Node(
        package=package,
        executable="single_pick_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
        respawn=LaunchConfiguration('enable_respawn'),
        respawn_delay=2.0
    )

    return LaunchDescription([enable_respawn_arg, node])
