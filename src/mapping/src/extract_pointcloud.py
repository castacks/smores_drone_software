#!/usr/bin/env python3

import os
import sys
import sqlite3
import argparse
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import ctypes
import struct


def convert_pointcloud2_to_open3d(ros_point_cloud):
    """Convert ROS PointCloud2 message to Open3D PointCloud."""
    # Extract points from PointCloud2 message
    points = np.array(list(pc2.read_points(ros_point_cloud, skip_nans=True)))

    # Check if the point cloud has RGB information
    has_rgb = False
    for field in ros_point_cloud.fields:
        if field.name == 'rgb' or field.name == 'rgba':
            has_rgb = True
            break

    # Create Open3D point cloud
    o3d_pcd = o3d.geometry.PointCloud()

    # Set points (assuming first 3 values are x, y, z)
    if points.shape[1] >= 3:
        o3d_pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])

    # Set colors if available
    if has_rgb and points.shape[1] >= 4:
        colors = []
        for point in points:
            # Extract RGB from the point cloud
            rgb = point[3]
            # Convert RGB value to r, g, b components
            s = struct.pack('>f', rgb)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            colors.append([r / 255.0, g / 255.0, b / 255.0])

        o3d_pcd.colors = o3d.utility.Vector3dVector(colors)

    return o3d_pcd


def check_directory(directory_path):
    """Check if directory exists and is empty."""
    # Check if directory exists
    if not os.path.isdir(directory_path):
        print(f"Error: Directory '{directory_path}' does not exist.")
        return False

    # Check if directory is empty
    if os.listdir(directory_path):
        print(f"Error: Directory '{directory_path}' is not empty.")
        return False

    return True


def extract_pointclouds_from_bag(bag_path, topic_name, output_dir):
    """Extract PointCloud2 messages from a ROS2 bag file and save them as PCD files."""
    # Connect to the SQLite database
    conn = sqlite3.connect(bag_path)
    cursor = conn.cursor()

    # Get topic id
    cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
    topic_id_result = cursor.fetchone()

    if not topic_id_result:
        print(f"Error: Topic '{topic_name}' not found in the bag file.")
        conn.close()
        return False

    topic_id = topic_id_result[0]

    # Get message type
    cursor.execute("SELECT type FROM topics WHERE id = ?", (topic_id,))
    topic_type = cursor.fetchone()[0]

    # Verify that the topic contains PointCloud2 messages
    if topic_type != 'sensor_msgs/msg/PointCloud2':
        print(f"Error: Topic '{topic_name}' does not contain PointCloud2 messages. Found: {topic_type}")
        conn.close()
        return False

    # Get message type and deserializer
    msg_type = get_message(topic_type)

    # Query all messages for the specified topic
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
    messages = cursor.fetchall()

    if not messages:
        print(f"No messages found for topic '{topic_name}'.")
        conn.close()
        return False

    print(f"Found {len(messages)} PointCloud2 messages for topic '{topic_name}'.")

    # Process each message
    for i, (timestamp, data) in enumerate(messages):
        # Deserialize the message
        ros_msg = deserialize_message(data, msg_type)

        # Convert to Open3D PointCloud
        o3d_pcd = convert_pointcloud2_to_open3d(ros_msg)

        # Save as PCD file
        output_file = os.path.join(output_dir, f"pointcloud_{timestamp}.pcd")
        o3d.io.write_point_cloud(output_file, o3d_pcd)

        print(f"Saved {output_file} ({i + 1}/{len(messages)})")

    conn.close()
    return True


def main():
    parser = argparse.ArgumentParser(description='Extract PointCloud2 messages from a ROS2 bag file.')
    parser.add_argument('bag_path', help='Path to the ROS2 bag file (.db3)')
    parser.add_argument('topic_name', help='Name of the topic containing PointCloud2 messages')
    parser.add_argument('output_dir', help='Directory to save the extracted PCD files')

    args = parser.parse_args()

    # Check if the bag file exists
    if not os.path.isfile(args.bag_path):
        print(f"Error: Bag file '{args.bag_path}' does not exist.")
        return 1

    # Check if the output directory exists and is empty
    if not check_directory(args.output_dir):
        return 1

    # Extract PointCloud2 messages
    success = extract_pointclouds_from_bag(args.bag_path, args.topic_name, args.output_dir)

    if success:
        print("PointCloud extraction completed successfully.")
        return 0
    else:
        print("PointCloud extraction failed.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
