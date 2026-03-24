from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('amorph_bringup'),
        'launch'
    )
    
    return LaunchDescription([
        # 启动能量单元定位节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'eu_locator.launch.py')
            ),
            launch_arguments={'enable_respawn': 'true'}.items(),
        ),
        
        # 启动G336L相机
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'g336l.launch.py')
            ),
            launch_arguments={'enable_respawn': 'true'}.items(),
        ),
        
        # 启动MoveIt
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'moveit.launch.py')
            ),
            launch_arguments={'enable_respawn': 'true'}.items(),
        ),
        
        # 启动主控节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'master.launch.py')
            ),
            launch_arguments={'enable_respawn': 'true'}.items(),
        ),
        
        # # 启动传输节点
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(launch_dir, 'transport.launch.py')
        #     ),
        #     launch_arguments={'enable_respawn': 'true'}.items(),
        # ),
    ])
