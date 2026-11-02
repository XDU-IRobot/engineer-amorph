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

    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.85',
        description='YOLO detection confidence threshold'
    )

    track_max_age_arg = DeclareLaunchArgument(
        'track_max_age',
        default_value='15',
        description='Frames a lost track is kept alive (Kalman predicted) before removal'
    )

    track_min_hits_arg = DeclareLaunchArgument(
        'track_min_hits',
        default_value='3',
        description='Consecutive hits needed before a track is published'
    )

    track_mahalanobis_thresh_arg = DeclareLaunchArgument(
        'track_mahalanobis_thresh',
        default_value='15.0',
        description='3D Mahalanobis distance gate for association'
    )

    track_iou_weight_arg = DeclareLaunchArgument(
        'track_iou_weight',
        default_value='0.4',
        description='Weight of 2D box IoU in the association cost (rest is 3D distance)'
    )

    locator_node = Node(
        package="amorph_vision",
        executable="eu_locator_v3_node",
        name="eu_locator_node",
        namespace="",
        output="screen",
        respawn=LaunchConfiguration('enable_respawn'),
        respawn_delay=2.0,
        parameters=[{
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'track_max_age': LaunchConfiguration('track_max_age'),
            'track_min_hits': LaunchConfiguration('track_min_hits'),
            'track_mahalanobis_thresh': LaunchConfiguration(
                'track_mahalanobis_thresh'),
            'track_iou_weight': LaunchConfiguration('track_iou_weight'),
        }],
    )

    return LaunchDescription([
        enable_respawn_arg,
        conf_threshold_arg,
        track_max_age_arg,
        track_min_hits_arg,
        track_mahalanobis_thresh_arg,
        track_iou_weight_arg,
        locator_node,
    ])
