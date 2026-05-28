from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 第1点像素坐标
    point1_x_arg = DeclareLaunchArgument(
        'point1_x',
        default_value='422',
        description='X pixel coordinate of point 1'
    )
    point1_y_arg = DeclareLaunchArgument(
        'point1_y',
        default_value='529',
        description='Y pixel coordinate of point 1'
    )

    # 第2点像素坐标
    point2_x_arg = DeclareLaunchArgument(
        'point2_x',
        default_value='840',
        description='X pixel coordinate of point 2'
    )
    point2_y_arg = DeclareLaunchArgument(
        'point2_y',
        default_value='529',
        description='Y pixel coordinate of point 2'
    )

    # 窗口大小
    window_size_arg = DeclareLaunchArgument(
        'window_size',
        default_value='10',
        description='Square window side length in pixels for averaging'
    )

    # Range 消息参数
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_depth_frame',
        description='TF frame ID for Range messages'
    )

    two_point_node = Node(
        package='amorph_vision',
        executable='two_point_range',
        name='two_point_range',
        namespace='',
        parameters=[{
            'point1_x': LaunchConfiguration('point1_x'),
            'point1_y': LaunchConfiguration('point1_y'),
            'point2_x': LaunchConfiguration('point2_x'),
            'point2_y': LaunchConfiguration('point2_y'),
            'window_size': LaunchConfiguration('window_size'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        point1_x_arg,
        point1_y_arg,
        point2_x_arg,
        point2_y_arg,
        window_size_arg,
        frame_id_arg,
        two_point_node,
    ])
