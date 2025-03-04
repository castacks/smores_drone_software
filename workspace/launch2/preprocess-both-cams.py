from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='thermal-preprocessing',
            # namespace='thermal_right',
            executable='thermal-preprocessing',
            name='thermal_left_preprocessor',
            remappings=[
                ('/thermal_raw', '/thermal_left/image'),
                ('preprocessed', '/thermal_left/preprocessed_image'),
            ]
        ),
         Node(
            package='thermal-preprocessing',
            # namespace='thermal_right',
            executable='thermal-preprocessing',
            name='thermal_right_preprocessor',
            remappings=[
                ('/thermal_raw', '/thermal_right/image'),
                ('preprocessed', '/thermal_right/preprocessed_image'),
            ]
        ),
    ])