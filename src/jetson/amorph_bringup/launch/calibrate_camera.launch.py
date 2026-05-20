from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='cameracalibrator',
            output='screen',
            arguments=['--size', '11x8', '--square', '0.03', '--no-service-check'],
            remappings=[
                ('image', '/vt_camera/image_raw'),
                ('camera', '/vt_camera'),
            ],
        ),
    ])
