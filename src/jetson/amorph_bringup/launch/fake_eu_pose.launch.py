from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    child_frame_arg = DeclareLaunchArgument("child_frame_id", default_value="energy_unit_selected", description="TF child frame name")

    fake_eu = Node(
        package="amorph_vision",
        executable="fake_eu_marker",
        name="fake_eu_marker",
        output="screen",
        parameters=[{
            "child_frame_id": LaunchConfiguration("child_frame_id"),
        }],
    )

    return LaunchDescription([
        child_frame_arg,
        fake_eu,
    ])
