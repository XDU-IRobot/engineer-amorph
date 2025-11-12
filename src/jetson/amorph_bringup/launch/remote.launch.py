from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
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
                            "enableChannelProportionalMax": 1.0,}],
                output="screen"),
        ],
    )