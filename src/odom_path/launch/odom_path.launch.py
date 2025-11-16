from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for odom_path_node with remappable topics.

    Usage:
        ros2 launch odom_path odom_path.launch.py
        ros2 launch odom_path odom_path.launch.py odom_topic:=/custom/odom path_topic:=/custom/path
    """

    # Declare launch arguments
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/mavros/odometry/out',
        description='Input odometry topic'
    )

    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        default_value='odom_path',
        description='Output path topic'
    )

    max_path_length_arg = DeclareLaunchArgument(
        'max_path_length',
        default_value='1000',
        description='Maximum number of poses to keep in path'
    )

    # Create node
    odom_path_node = Node(
        package='odom_path',
        executable='odom_path_node',
        name='odom_path_node',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'path_topic': LaunchConfiguration('path_topic'),
            'max_path_length': LaunchConfiguration('max_path_length'),
        }]
    )

    return LaunchDescription([
        odom_topic_arg,
        path_topic_arg,
        max_path_length_arg,
        odom_path_node,
    ])
