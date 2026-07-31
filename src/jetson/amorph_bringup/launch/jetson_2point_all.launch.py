from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('amorph_bringup'),
        'launch'
    )

    return LaunchDescription([
        # G336L 相机驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'g336l.launch.py')
            ),
        ),

        # 双点测距节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, '2point.launch.py')
            ),
        ),

        # CAN FD 传输节点：将测距数据广播到总线上
        Node(
            package='amorph_onboard_transport',
            executable='two_point_range_broadcaster',
            name='two_point_range_broadcaster',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
