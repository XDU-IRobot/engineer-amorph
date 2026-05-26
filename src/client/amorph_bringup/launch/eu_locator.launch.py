from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_respawn_arg = DeclareLaunchArgument(
        'enable_respawn',
        default_value='false',
        description='Enable automatic respawn on failure (for production use)'
    )

    locator_node = Node(
        package="amorph_vision",
        executable="locator_v2",
        name="energy_unit_locator_node",
        namespace="",
        parameters=[{
            "model_path": "",
            "confidence_threshold": 0.5,
            "erosion_kernel_size": 5,
            "erosion_iterations": 1,
            "publish_masked_pointcloud": False,
            "publish_annotated_image": True,
            "smoothing_alpha": 0.3,
            "tracker_max_age": 15,
            "tracker_min_hits": 1,
            "tracker_mahalanobis_thresh": 15.0,
            "tracker_iou_weight": 0.4,
        }],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        output="screen",
        respawn=LaunchConfiguration('enable_respawn'),
        respawn_delay=2.0
    )

    return LaunchDescription([
        enable_respawn_arg,
        locator_node,
    ])
