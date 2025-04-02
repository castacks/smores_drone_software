from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directories
    epson_imu_share = get_package_share_directory('epson_imu_driver_ros2')
    thermal_preprocess_share = get_package_share_directory('thermal-preprocessing')
    thermal_publisher_share = get_package_share_directory('thermal-publisher')

    # Launch Epson IMU driver
    epson_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(epson_imu_share, 'launch', 'epson_g365_fixed_launch.py')
        )
    )

    # Launch thermal preprocessing
    thermal_preprocess_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(thermal_preprocess_share, 'launch', 'preprocess-both-cams.py')
        )
    )

    # Launch thermal publisher
    thermal_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(thermal_publisher_share, 'launch', 'both_cameras_launch.py')
        )
    )

    return LaunchDescription([
        epson_imu_launch,
        thermal_preprocess_launch,
        thermal_publisher_launch
    ])
