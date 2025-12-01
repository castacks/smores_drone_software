from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rovtio',
            executable='rovtio_node',
            name='rovtio',
            output='screen',
            remappings=[
                ('/rovtio/odometry', '/rovtio/odometry'),
                # Uncomment as needed
                # ('/imu0', '/vn100/imu'),
                # ('/imu0', '/matrice/imu'),
            ],
            parameters=[
                {'filter_config': '/home/aayush/prj/ROVTIO/workspace/src/rovtio/cfg/rovtio/rovtio.info'},
                {'imu_topic': '/vn100/imu'},
                {'camera_topic0': '/thermal_left/image'},
                {'camera0_config': '/home/aayush/prj/ROVTIO/workspace/src/rovtio/cfg/rovtio/charlie_visual.yaml'},
                {'cam0_offset': 0.0},
                {'use_sim_time': True},
                {'cam1_offset': -0.02414188675155223},
                {'maxDelayBeforeDropping': -0.2},
                {'storeRuntimes': False},
                {'maxTimeCamInactive': 0.12},
            ]
        )
    ])
