#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import os
import open3d as o3d

class PointCloudSaverNode(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        
        # Declare parameters with default values
        self.declare_parameter('topic', '/global_map')
        self.declare_parameter('output_prefix', '/external/data/ply')
        self.declare_parameter('counter', 'False')

        # Get parameters
        self.topic = self.get_parameter('topic').value
        self.output_prefix = self.get_parameter('output_prefix').value
        self.is_counter = (self.get_parameter('counter').value) == 'True'
        print(f"Topic is {self.topic} and outputting to {self.output_prefix}")
        
        # Counter for filenames
        self.counter = 0
        
        # Create subscription
        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic,
            self.pointcloud_callback,
            1)
        
        self.get_logger().info(f'Subscribed to {self.topic}')
        self.get_logger().info(f'Saving PLY files with prefix: {self.output_prefix}')
    

    def pointcloud_callback(self, msg):
        self.get_logger().info(f'Callback came through')
    
        # Convert the data from array.array to numpy array first
        data_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    
        points = []
        # Extract x, y, z from the binary data
        for point_data in data_array:
            x = np.frombuffer(point_data[0:4].tobytes(), dtype=np.float32)[0]
            y = np.frombuffer(point_data[4:8].tobytes(), dtype=np.float32)[0]
            z = np.frombuffer(point_data[8:12].tobytes(), dtype=np.float32)[0]
        
            # Skip NaN points
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                points.append([x, y, z])
    
        # Skip if no valid points
        if not points:
            self.get_logger().warn('Received empty point cloud. Skipping.')
            return

        # Convert to numpy array
        points_array = np.array(points)
    
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_array)
    
        # Create filename with counter
        filename = f"{self.output_prefix}_{self.counter:04d}.ply"
        if self.is_counter:
            self.counter += 1
    
        # Save PLY file
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f'Saved {len(points)} points to: {os.path.abspath(filename)}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
