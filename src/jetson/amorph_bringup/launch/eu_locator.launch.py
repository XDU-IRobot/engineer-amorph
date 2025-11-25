from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="energy_unit_locator",
                executable="locator_v2",
                name="energy_unit_locator_node",
                parameters=[{
                    #
                }],
                output="screen"),
        ],
    )
