#!/usr/bin/env python3

import os
import argparse
import cv2
import numpy as np
from cv_bridge import CvBridge
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader
import sensor_msgs.msg as sensor_msgs

def extract_images(bag_path, output_dir, image_topic):
    """
    Extract images from a ROS2 bag file and save them to a specified directory.
    
    Args:
        bag_path (str): Path to the ROS2 bag file directory
        output_dir (str): Directory where images will be saved
        image_topic (str): ROS2 topic containing image messages
    """
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created directory: {output_dir}")
    
    # Initialize CV bridge for converting between ROS and OpenCV images
    bridge = CvBridge()
    
    # Initialize ROS2 reader
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    
    # Create a map of topic to type
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    
    # Check if the requested topic exists in the bag
    if image_topic not in type_map:
        available_topics = [t.name for t in topic_types]
        print(f"Error: Topic '{image_topic}' not found in the bag file.")
        print(f"Available topics: {available_topics}")
        return
    
    # Check if the topic is actually an image topic
    if not type_map[image_topic].startswith("sensor_msgs/msg/Image"):
        print(f"Error: Topic '{image_topic}' is not an image topic.")
        print(f"Topic type: {type_map[image_topic]}")
        return
    
    # Start reading messages
    count = 0
    
    print(f"Extracting images from topic: {image_topic}")
    
    while reader.has_next():
        try:
            # Get the next message
            (topic, data, t) = reader.read_next()
            
            # Skip if not the desired topic
            if topic != image_topic:
                continue
            
            # Get the message type and deserialize
            msg_type = get_message(type_map[topic])
            from rclpy.serialization import deserialize_message
            msg = deserialize_message(data, msg_type)
            
            # Convert the ROS image message to OpenCV format
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            except Exception as e:
                print(f"Error converting image message: {e}")
                continue
            
            # Generate a filename using the message timestamp
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                timestamp = msg.header.stamp
                secs = timestamp.sec
                nsecs = timestamp.nanosec
            else:
                # Use the storage timestamp if header timestamp not available
                secs = t // 10**9
                nsecs = t % 10**9
            
            filename = f"{output_dir}/frame_{count}.png"
            
            # Save the image
            cv2.imwrite(filename, cv_image)
            count += 1
            
            # Print progress
            if count % 100 == 0:
                print(f"Extracted {count} images so far")
        
        except Exception as e:
            print(f"Error processing message: {e}")
    
    print(f"Extraction complete. {count} images saved to {output_dir}")

def main():
    parser = argparse.ArgumentParser(description="Extract images from a ROS2 bag file")
    parser.add_argument("bag_path", help="Path to the ROS2 bag file directory")
    parser.add_argument("output_dir", help="Directory where images will be saved")
    parser.add_argument("--topic", default="/camera/image_raw", 
                      help="ROS2 topic containing image messages (default: /camera/image_raw)")
    parser.add_argument("--format", default="png", choices=["png", "jpg", "bmp"],
                      help="Output image format (default: png)")
    
    args = parser.parse_args()
    
    extract_images(args.bag_path, args.output_dir, args.topic)

if __name__ == "__main__":
    main()
