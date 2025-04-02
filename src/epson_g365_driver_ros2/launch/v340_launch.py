"""Launch file for Epson V340PDD0 imu_node for ess_imu_ros2_uart_driver package"""

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial port name",
            ),
            DeclareLaunchArgument(
                name="frame_id",
                default_value="imu_link",
                description="imu message frame_id field",
            ),
            DeclareLaunchArgument(
                name="imu_topic",
                default_value="/epson_imu/data_raw",
                description="topic name for publishing imu messages.",
            ),
            DeclareLaunchArgument(
                name="burst_polling_rate",
                default_value="4000.0",
                description="Set to atleast 2x the output rate of IMU. Should not need to change.",
            ),
            DeclareLaunchArgument(
                name="imu_dout_rate",
                # value: output rate (Hz)  Recommended Filter
                # 1: 1000                    TAP>=1
                # 2: 500                     TAP>=2
                # 3: 250                     TAP>=4
                # 4: 125                     TAP>=8
                # 5: 62.5                    TAP>=16
                # 6: 31.25                   TAP=32
                default_value="4",
                description="Sets data output rate of IMU",
            ),
            DeclareLaunchArgument(
                name="imu_filter_sel",
                # value: Filter Setting
                # 2: Moving Average TAP1
                # 3: Moving Average TAP2
                # 4: Moving Average TAP4
                # 5: Moving Average TAP8
                # 6: Moving Average TAP16
                # 7: Moving Average TAP32
                default_value="5",
                description="Sets the IMU filter",
            ),
            DeclareLaunchArgument(
                name="time_correction_en",
                default_value="false",
                description="Enables using IMU external counter reset function for timestamp with external 1PPS connected to IMU input pin for GPIO2/EXT",
            ),
            DeclareLaunchArgument(
                name="ext_trigger_en",
                default_value="false",
                description="Enables using IMU external trigger function for sending IMU samples with external trigger signal connected to IMU input pin for GPIO2/EXT",
            ),
            launch_ros.actions.Node(
                package="ess_imu_ros2_uart_driver",
                executable="ess_imu_uart_node",
                output="screen",
                parameters=[
                    {
                        "__log_level": "INFO",
                        "serial_port": LaunchConfiguration("serial_port"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "burst_polling_rate": LaunchConfiguration("burst_polling_rate"),
                        "imu_dout_rate": LaunchConfiguration("imu_dout_rate"),
                        "imu_filter_sel": LaunchConfiguration("imu_filter_sel"),
                        "time_correction_en": LaunchConfiguration("time_correction_en"),
                        "ext_trigger_en": LaunchConfiguration("ext_trigger_en"),
                    }
                ],
            ),
        ]
    )
