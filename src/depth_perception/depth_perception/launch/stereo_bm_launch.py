from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    foxglove_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"
        ]),
        launch_arguments={
            "port": "8765",
            "address": "0.0.0.0",
        }.items(),
    )
    stereobm_node = Node(
        package="depth_perception",
        executable="stereobm",
        output="screen",
    )
    return LaunchDescription(
        [
            stereobm_node
        ]
    )
