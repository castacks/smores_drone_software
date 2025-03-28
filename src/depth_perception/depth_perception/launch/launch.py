from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

# colcon build --packages-select depth_perception && source install/setup.bash && ros2 launch depth_perception launch.py

def generate_launch_description():
    preproc_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("thermal-preprocessing"), "launch", "preprocess-both-cams.py"
        ])
    )

    mogeinf_left = Node(
        package="depth_perception",  
        executable="mono_pub_sub",  
        namespace='thermal_left',
        remappings=[
            ('thermal/image', 'preprocd_image'),
            ('thermal/moge/depthmap', 'moge/depthmap'),
            ('thermal/moge', 'moge')
        ],
        output="screen",
    )
    
    mogeinf_right = Node(
        package="depth_perception",  
        executable="mono_pub_sub",  
        namespace='thermal_right',
        remappings=[
            ('thermal/image', 'preprocd_image'),
            ('thermal/moge/depthmap', 'moge/depthmap'),
            ('thermal/moge', 'moge')
        ],
    )

    madpose_solver = Node(
        package="depth_perception", 
        executable="madpose",  
    )

    pcl_sub_test = Node(
        package="depth_perception",
        executable="pclsub",
    )

    foxglove_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"
        ]),
        launch_arguments={
            "port": "8765", 
            "address": "0.0.0.0",
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
            preproc_launch,
            mogeinf_left,
            mogeinf_right,
            madpose_solver,
            #pcl_sub_test,
            #foxglove_launch,
        ]
    )
