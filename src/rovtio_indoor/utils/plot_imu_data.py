#!/usr/bin/env python3

import argparse
import matplotlib.pyplot as plt
import sqlite3
import sys
import yaml

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description='Plot IMU data from a ROS2 bag file.')
    parser.add_argument('bag_file', help='Path to the ROS2 bag file (directory).')
    parser.add_argument('imu_topic', help='Name of the IMU topic (e.g., /imu0).')
    args = parser.parse_args()

    bag_path = args.bag_file
    topic_name = args.imu_topic

    # ROS 2 bags are sqlite3 databases. We can read the data directly from the database.
    # This is a simplified approach. A more robust solution would use the rosbag2_py API,
    # but that is more complex and requires the user to have the rosbag2_py library installed.

    # The database file is specified in the metadata.yaml file.
    try:
        with open(f"{bag_path}/metadata.yaml", 'r') as f:
            metadata = yaml.safe_load(f)
            if 'rosbag2_bagfile_information' in metadata: # ROS2 Foxy and newer
                db_filename = metadata['rosbag2_bagfile_information']['relative_file_paths'][0]
            elif 'rosbag2_storage_information' in metadata: # ROS2 Dashing
                db_filename = metadata['rosbag2_storage_information']['relative_file_paths'][0]
            else:
                raise KeyError("Could not find rosbag storage information in metadata.yaml")
            conn = sqlite3.connect(f"{bag_path}/{db_filename}")
    except Exception as e:
        print(f"Error opening database: {e}", file=sys.stderr)
        sys.exit(1)

    cursor = conn.cursor()

    # Get the topic ID and message type from the topics table.
    try:
        cursor.execute("SELECT id, type FROM topics WHERE name = ?", (topic_name,))
        topic_id, topic_type = cursor.fetchone()
    except TypeError:
        print(f"Topic '{topic_name}' not found in the bag file.", file=sys.stderr)
        sys.exit(1)

    # Get the message definition.
    try:
        msg_type = get_message(topic_type)
    except (AttributeError, ModuleNotFoundError, ValueError):
        print(f"Could not find message type '{topic_type}'. Make sure your ROS2 environment is sourced.", file=sys.stderr)
        sys.exit(1)

    # Read the messages from the messages table.
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
    messages = cursor.fetchall()

    timestamps = []
    accel_x = []
    accel_y = []
    accel_z = []
    gyro_x = []
    gyro_y = []
    gyro_z = []

    # Deserialize the messages and store the data.
    for timestamp, data in messages:
        msg = deserialize_message(data, msg_type)
        timestamps.append(timestamp)
        accel_x.append(msg.linear_acceleration.x)
        accel_y.append(msg.linear_acceleration.y)
        accel_z.append(msg.linear_acceleration.z)
        gyro_x.append(msg.angular_velocity.x)
        gyro_y.append(msg.angular_velocity.y)
        gyro_z.append(msg.angular_velocity.z)

    conn.close()

    # Plot the data.
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

    ax1.plot(timestamps, accel_x, label='x')
    ax1.plot(timestamps, accel_y, label='y')
    ax1.plot(timestamps, accel_z, label='z')
    ax1.set_title('Accelerometer')
    ax1.set_ylabel('m/s^2')
    ax1.legend()
    ax1.grid(True)

    ax2.plot(timestamps, gyro_x, label='x')
    ax2.plot(timestamps, gyro_y, label='y')
    ax2.plot(timestamps, gyro_z, label='z')
    ax2.set_title('Gyroscope')
    ax2.set_ylabel('rad/s')
    ax2.set_xlabel('Timestamp (ns)')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
