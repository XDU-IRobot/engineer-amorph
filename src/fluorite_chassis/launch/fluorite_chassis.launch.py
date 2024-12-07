import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

CONFIG_FILES = [
    'settings.yaml',
]

def generate_launch_description():
    # 检查CONFIG_FILES中要求的的配置文件是否存在，不存在就退出
    config_dir = os.path.join(get_package_share_directory('fluorite_chassis'), 'config')
    for file in CONFIG_FILES:
        if not os.path.exists(os.path.join(config_dir, file)):
            print(f"[ERROR] [{os.path.basename(__file__)}]: Required configuration file config/{file} does not exist")
            os._exit(1)
    config_file_full_path = [os.path.join(config_dir, file) for file in CONFIG_FILES]

    return LaunchDescription(
        [
            Node(
                package='fluorite_chassis',
                executable='fluorite_chassis',
                name='fluorite_chassis',
                output='screen',
                parameters=config_file_full_path,
            )
        ]
    )