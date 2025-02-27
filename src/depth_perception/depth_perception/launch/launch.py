from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    mogeinf_left = Node(
                package="depth_perception",  # Replace with your package name
                executable="mono_pub_sub",  # Replace with your node executable name
                name="moge_pub_sub",
                namespace='thermal_left',
                remappings=[
                    ('thermal/image', 'image'),  
                    ('thermal/preproc', 'preproc_image'),
                    ('thermal/moge/depthmap', 'moge_depthmap')
                ],
                output="screen",
            )
    
    mogeinf_right = Node(
                package="depth_perception",  # Replace with your package name
                executable="mono_pub_sub",  # Replace with your node executable name
                name="moge_pub_sub",
                namespace='thermal_right',
                remappings=[
                    ('thermal/image', 'image'),  
                    ('thermal/preproc', 'preproc_image'),
                    ('thermal/moge/depthmap', 'moge_depthmap')
                ],
                output="screen",
            )

    foxglove_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"
        ]),
        launch_arguments={
            "port": "8765", 
            "address": "0.0.0.0",
            # "send_buffer_limit": "50000000" # temp
            }.items(),
    )
    return LaunchDescription(
        [
            # First instance of ThermalPubSub for thermal_left/image
            #    Node(
            #        package='depth_perception',  # Replace with your package name
            #        executable='thermal_pub_sub',  # Replace with your node executable name
            #        name='thermal_pub_sub_left',
            #        # namespace='thermal_preprocessed',
            #        remappings=[
            #            ('thermal/image', 'thermal_left/image'),
            #            ('thermal_preprocessed/image', 'thermal_preprocessed_left/image')
            #        ],
            #        output='screen'
            #    ),
            #    # Second instance of ThermalPubSub for thermal_right/image
            #    Node(
            #        package='depth_perception',  # Replace with your package name
            #        executable='thermal_pub_sub',  # Replace with your node executable name
            #        name='thermal_pub_sub_right',
            #        # namespace='thermal_preprrocessed',
            #        remappings=[
            #            ('thermal/image', 'thermal_right/image'),  # Remap input topic
            #            ('thermal_preprocessed/image', 'thermal_preprocessed_right/image')  # Remap output topic
            #        ],
            #        output='screen'
            #    ),
            mogeinf_left,
            #mogeinf_right,
            foxglove_launch,
        ]
    )
