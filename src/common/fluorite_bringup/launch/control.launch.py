import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{
                    "rate": 200,
                    "source_list": [
                        # "fluorite/chassis/joint_states",
                        "fluorite/arm/joint_states",
                    ]
                }],
            ),
            Node(
                package="witmotion_ros2_driver",
                executable="witmotion_ros2_driver",
                name="witmotion_ros2_driver",
                parameters=[{"serial_port": "/dev/ttyS4",
                             "baud_rate": 921600, }],
                output="screen"),
            Node(
                package="sbus_bridge",
                executable="sbus_bridge_node",
                name="sbus_bridge_node",
                parameters=[{"port": "/dev/ttyS0",
                             "refresh_rate_hz": 5,
                             "rxMinValue": 353,
                             "rxMaxValue": 1695,
                             "outMinValue": -1000,
                             "outMaxValue": 1000,
                             "silentOnFailsafe": False,
                             "enableChannelNum": -1,
                             "enableChannelProportionalMin": 0.0,
                             "enableChannelProportionalMax": 1.0, }],
                output="screen"),
            Node(
                package="fluorite_control",
                executable="fluorite_control_chassis",
                name="fluorite_control_chassis",
                parameters=[os.path.join(get_package_share_directory(
                    'fluorite_control'), "config", "settings.yaml")],
                output="screen"),
            Node(
                package="fluorite_control",
                executable="fluorite_control_arm",
                name="fluorite_control_arm",
                parameters=[os.path.join(get_package_share_directory(
                    'fluorite_control'), "config", "settings.yaml")],
                output="screen"),
        ],
    )
