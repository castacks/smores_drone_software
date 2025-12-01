import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import argparse
import os


class SavePCDNode(Node):
    def __init__(self, topic_name, output_file):
        super().__init__('save_pcd_node')
        self.output_file = output_file
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self.listener_callback,
            1)
        self.get_logger().info(f"Subscribed to topic: {topic_name}")

    def listener_callback(self, msg: PointCloud2):
        # self.get_logger().info("Received PointCloud2 message. Converting...")

        points = np.array([
            [x, y, z] for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])

        if len(points) == 0:
            self.get_logger().warn("Received empty point cloud. Not saving.")
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(self.output_file, pcd)

        self.get_logger().info(f"Saved point cloud to {os.path.abspath(self.output_file)}")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Allow CLI args
    parser = argparse.ArgumentParser(description='Save PointCloud2 to PLY')
    parser.add_argument('--topic', type=str, default='/octomap/point/cloud/centers', help='Topic name to subscribe to')
    parser.add_argument('--output', type=str, default='output.ply', help='Output .ply file name')
    parsed_args, _ = parser.parse_known_args()

    node = SavePCDNode(parsed_args.topic, parsed_args.output)
    rclpy.spin(node)
