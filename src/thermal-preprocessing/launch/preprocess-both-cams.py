from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='thermal-preprocessing',
            # namespace='thermal_right',
            executable='thermal-preprocessing',
            name='thermal_left_preprocessor',
            parameters=[
                    {
                        "camIntrinsicsFile": "/workspace/smores_drone_software/calibrations/ORDv1_Smores_Feb2025/left_thermal.yaml",
                        "cam": "left"
                    }
                ]
        ),
         Node(
            package='thermal-preprocessing',
            # namespace='thermal_right',
            executable='thermal-preprocessing',
            name='thermal_right_preprocessor',
            parameters=[
                    {
                        "camIntrinsicsFile": "/workspace/smores_drone_software/calibrations/ORDv1_Smores_Feb2025/right_thermal.yaml", 
                        "cam": "right"
                    }
                ]
        ),
    ])