import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('amorph_bringup'),
        'launch'
    )

    # 参数名与 RosParams 结构体一致：嵌套结构体展开成 model.conf_threshold 这种形式
    args = [
        DeclareLaunchArgument('enable_respawn', default_value='false',
                              description='Enable automatic respawn on failure'),
        DeclareLaunchArgument('engine_path', default_value='',
                              description='TensorRT engine path; empty means <share>/resource/best.engine'),
        DeclareLaunchArgument('model.conf_threshold', default_value='0.85',
                              description='Confidence threshold applied on top of the engine-side threshold'),
        DeclareLaunchArgument('tracking.max_age', default_value='15',
                              description='Frames a lost track is kept alive before removal'),
        DeclareLaunchArgument('tracking.min_hits', default_value='20',
                              description='Consecutive hits needed before a track is published'),
        DeclareLaunchArgument('tracking.mahalanobis_thresh', default_value='15.0',
                              description='3D Mahalanobis distance gate for association'),
        DeclareLaunchArgument('tracking.iou_weight', default_value='0.4',
                              description='Weight of 2D box IoU in the association cost'),
        DeclareLaunchArgument('tracking.process_var', default_value='1.0',
                              description='KF process noise; larger = faster velocity tracking, less overshoot'),
        DeclareLaunchArgument('tracking.measurement_var', default_value='1e-3',
                              description='KF measurement variance (m^2); also sets the Mahalanobis gate radius'),
        DeclareLaunchArgument('log_timing', default_value='false',
                              description='Log per-frame YOLO / PCA+ICP timings'),
        DeclareLaunchArgument('publish_debug_clouds', default_value='false',
                              description='Publish PCA/ICP intermediate clouds on debug_clouds/det<i>/...'),
        DeclareLaunchArgument('debug_cloud_stride', default_value='4',
                              description='Subsample stride for debug point clouds (larger = less bandwidth)'),
    ]

    parameters = [{'engine_path': LaunchConfiguration('engine_path')}]
    for name in ('model.conf_threshold', 'tracking.max_age', 'tracking.min_hits',
                 'tracking.mahalanobis_thresh', 'tracking.iou_weight',
                 'tracking.process_var', 'tracking.measurement_var',
                 'log_timing',
                 'publish_debug_clouds', 'debug_cloud_stride'):
        parameters[0][name] = LaunchConfiguration(name)

    apriltag_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'apriltag.launch.py')
        )
    )

    locator_node = Node(
        package="amorph_vision_cpp",
        executable="amorph_vision_cpp_node",
        name="eu_locator_node",
        namespace="",
        output="screen",
        respawn=LaunchConfiguration('enable_respawn'),
        respawn_delay=2.0,
        parameters=parameters,
    )

    nodes = [apriltag_node, locator_node]

    return LaunchDescription(args + nodes)
