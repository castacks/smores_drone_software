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
                ('/rovtio/odometry', '/rovtio/odometry'),
                # Uncomment one of these as needed
                # ('/imu0', '/vn100/imu'),
                # ('/imu0', '/matrice/imu'),
            ],
            parameters=[
                {'filter_config': '/home/aayush/code/own-code/src/rovtio/cfg/phxpro/rovtio.copy.info'},
                {'imu_topic': '/epson_imu/data'},
                {'camera_topic0': '/thermal_left/image'},
                {'camera0_config': '/home/aayush/code/own-code/src/rovtio/cfg/phxpro/left-thermal.yaml'},
                {'camera_topic1': '/thermal_right/image'},
                {'camera1_config': '/home/aayush/code/own-code/src/rovtio/cfg/phxpro/right-thermal.yaml'},
                {'cam0_offset': 0.0},
                {'maxDelayBeforeDropping': -0.2},
                {'storeRuntimes': False},
                {'maxTimeCamInactive': 0.12},
            ],
            # prefix='gdbserver localhost:3000'
        )
    ])
