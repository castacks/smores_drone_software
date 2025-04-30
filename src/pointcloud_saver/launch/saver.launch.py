from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_saver',
            executable='saver_node',
            name='pointcloud_saver',
            output='screen',
            parameters=[{
                'topic': '/octomap_point_cloud_centers',  # Hardcoded topic
                'output_prefix': '/external/data/integratedMappingMetrics/ply',     # Hardcoded output prefix
                'counter':'False'
            }]
        )
    ])
