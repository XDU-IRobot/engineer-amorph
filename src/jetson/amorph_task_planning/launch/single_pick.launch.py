from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    offset_arg = DeclareLaunchArgument(
        'offset_distance', default_value='0.15',
        description='预抓取位置与目标中心的偏移距离 (m)')

    sphere_arg = DeclareLaunchArgument(
        'sphere_radius', default_value='0.02',
        description='可视化球体半径 (m)')

    pick_arg = DeclareLaunchArgument(
        'pick_target', default_value='l',
        description='抓取目标: l (左) 或 r (右)')

    node = Node(
        package='amorph_task_planning',
        executable='single_pick_node',
        name='single_pick_node',
        output='screen',
        parameters=[{
            'offset_distance': LaunchConfiguration('offset_distance'),
            'sphere_radius': LaunchConfiguration('sphere_radius'),
            'pick_target': LaunchConfiguration('pick_target'),
        }],
    )

    return LaunchDescription([offset_arg, sphere_arg, pick_arg, node])
