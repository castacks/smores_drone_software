from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='thermal-publisher',
            executable='thermal-publisher',
            name='thermal_publisher_left0',
            remappings=[
                ('/thermal_image', '/thermal_left/image'),
            ],
            parameters=[{
                "device_path": "/dev/flir_boson_video_69703"
            }]
        ),
        Node(
            package='thermal-publisher',
            executable='thermal-publisher',
            name='thermal_publisher_right',
            remappings=[
                ('/thermal_image', '/thermal_right/image'),
            ],
            parameters=[{
                "device_path": "/dev/flir_boson_video_34564"
            }]
        ),
  ])
