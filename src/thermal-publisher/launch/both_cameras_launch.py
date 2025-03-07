from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='thermal-publisher',
            executable='thermal-publisher',
            name='thermal_publisher_left',
            remappings=[
                ('/thermal_image', '/thermal_left/image'),
            ],
            parameters=[{
                "device_path": "/dev/flir_boson_video_34564"
            }]
        ),
        Node(
            package='thermal-publisher',
            executable='thermal-publisher',
            name='thermal_publisher_left',
            remappings=[
                ('/thermal_image', '/thermal_right/image'),
            ],
            parameters=[{
                "device_path": "/dev/flir_boson_video_69703"
            }]
        ),
  ])
