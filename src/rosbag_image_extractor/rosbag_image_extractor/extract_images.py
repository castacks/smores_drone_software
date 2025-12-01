#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
import os
import sys
from pathlib import Path
import subprocess
import time


class ImageExtractor(Node):
    def __init__(self, topic, output_dir, output_format, save_every=1, fps=30):
        super().__init__('image_extractor')

        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.output_format = output_format
        self.save_every = save_every
        self.fps = fps
        self.frame_count = 0
        self.images = []
        self.video_writer = None

        # Create output directory if it doesn't exist
        Path(output_dir).mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f'Subscribing to topic: {topic}')
        self.get_logger().info(f'Output directory: {output_dir}')
        self.get_logger().info(f'Output format: {output_format}')

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.output_format == 'images':
                # Save individual image
                if self.frame_count % self.save_every == 0:
                    filename = os.path.join(self.output_dir, f'frame_{self.frame_count:06d}.png')
                    cv2.imwrite(filename, cv_image)
                    self.get_logger().info(f'Saved frame {self.frame_count}: {filename}')

            elif self.output_format == 'mp4':
                # Initialize video writer on first frame
                if self.video_writer is None:
                    height, width = cv_image.shape[:2]
                    output_file = os.path.join(self.output_dir, 'output.mp4')
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    self.video_writer = cv2.VideoWriter(output_file, fourcc, self.fps, (width, height))
                    self.get_logger().info(f'Created video writer: {output_file} ({width}x{height} @ {self.fps}fps)')

                # Write frame to video
                self.video_writer.write(cv_image)
                self.get_logger().info(f'Wrote frame {self.frame_count} to video')

            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def cleanup(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f'Released video writer. Total frames: {self.frame_count}')


def play_rosbag(bag_file):
    """Play rosbag in the background"""
    cmd = ['ros2', 'bag', 'play', bag_file]
    return subprocess.Popen(cmd)


def main():
    parser = argparse.ArgumentParser(
        description='Extract images from a ROS2 bag file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Save as individual PNG images
  python3 extract_images.py -b /path/to/bag -t /camera/image_raw -o /output/dir -f images

  # Save as MP4 video at 30fps
  python3 extract_images.py -b /path/to/bag -t /camera/image_raw -o /output/dir -f mp4 --fps 30
        """)

    parser.add_argument('-b', '--bag', required=True, help='Path to ROS2 bag file')
    parser.add_argument('-t', '--topic', required=True, help='Image topic to extract')
    parser.add_argument('-o', '--output', required=True, help='Output directory')
    parser.add_argument('-f', '--format', choices=['images', 'mp4'], default='images',
                        help='Output format: "images" for individual PNGs or "mp4" for video')
    parser.add_argument('--save_every', type=int, default=1,
                        help='How often to save images (default: 1, every image)')
    parser.add_argument('--fps', type=int, default=30,
                        help='FPS for MP4 output (default: 30)')

    args = parser.parse_args()

    # Verify bag file exists
    if not os.path.exists(args.bag):
        print(f"Error: Bag file not found: {args.bag}")
        sys.exit(1)

    # Initialize ROS2
    rclpy.init()

    # Create image extractor node
    extractor = ImageExtractor(args.topic, args.output, args.format, args.save_every, args.fps)

    # Start playing the bag file
    print(f"Starting rosbag playback: {args.bag}")
    bag_process = play_rosbag(args.bag)

    try:
        # Spin the node to receive messages
        rclpy.spin(extractor)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        # Cleanup
        extractor.cleanup()
        extractor.destroy_node()
        rclpy.shutdown()

        # Terminate bag playback
        if bag_process.poll() is None:
            print("Stopping rosbag playback...")
            bag_process.terminate()
            bag_process.wait()

        print(f"\nExtraction complete. Total frames: {extractor.frame_count}")


if __name__ == '__main__':
    main()
