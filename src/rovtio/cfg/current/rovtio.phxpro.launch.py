from launch import LaunchDescription
from launch_ros.actions import Node
# Optional: for dynamic path lookup if you want later
# from ament_index_python.packages import get_package_share_directory
# import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rovtio',
            executable='rovtio_node',
            name='rovtio',
            output='screen',
                        # Uncomment the next line to launch under GDB if needed
            # prefix='konsole -e gdb --args',
            remappings=[
                ('/rovtio/odometry', '/mavros/odometry/out'),
                # Uncomment one of these as needed
                # ('/imu0', '/vn100/imu'),
                # ('/imu0', '/matrice/imu'),
            ],
            parameters=[
                {'filter_config': '/external/smores_drone_software/src/cfg/current/rovtio.phxpro.info'},
                {'imu_topic': '/epson_imu/data'},
                {'camera_topic0': '/thermal_left/image'},
                {'camera0_config': '/external/smores_drone_software/src/cfg/current/left-thermal-rovio.yaml'},
                {'camera_topic1': '/thermal_right/image'},
                {'camera1_config': '/external/smores_drone_software/src/cfg/current/right-thermal-rovio.yaml'},
                {'cam0_offset': 0.0},
                {'cam1_offset': 0.0},
                {'maxDelayBeforeDropping': -0.2},
                {'storeRuntimes': False},
                {'maxTimeCamInactive': 4.0},
            ],
            # prefix='gdbserver localhost:3000'
        )
    ])
