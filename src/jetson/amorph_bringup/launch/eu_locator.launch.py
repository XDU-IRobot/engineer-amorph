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
                    "model_path": "",
                    "confidence_threshold": 0.5,
                    "erosion_kernel_size": 5,
                    "erosion_iterations": 1,
                    "publish_masked_pointcloud": False,
                    "smoothing_alpha": 0.3,
                }],
                output="screen"),
        ],
    )
