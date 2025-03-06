import rclpy
import sys
sys.path.append("/opt/conda/lib/python3.10/site-packages/")
sys.path.append("/workspace/smores_drone_software/include/MoGe")
sys.path.append("/workspace/smores_drone_software/include/madpose")
sys.path.append("/workspace/smores_drone_software/include/ros2_numpy")

import open3d as o3d
import numpy as np

# ROS imports
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
import ros2_numpy

DUMMY_FIELD_PREFIX = '__'

class PCLSubscriber(Node):

    def __init__(self):
        super().__init__('pclsub')

        self.subscription = self.create_subscription(
            PointCloud2,
            "thermal/madpose/pointcloud_left",
            self.save_point_cloud,
            1,
        )
        self.subscription

        self.i = 0


    def save_point_cloud(self, msg):
        data = ros2_numpy.numpify(msg)
        #self.get_logger().info(f"{data.dtype.names}")
        x = data['x']
        y = data['y']
        z = data['z']
        points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
        self.get_logger().info(f"{points.shape}")
        #colors = data['rgb']

        # Convert the numpy array to an Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        #pcd.colors = o3d.utility.Vector3dVector(colors)

        # Save the point cloud to a file
        o3d.io.write_point_cloud(f"data/test/pclrx/{self.i}_point_cloud_0.ply", pcd)
        self.i+=1

        # o3d.visualization.draw_plotly([pcd])

def main(args=None):
   rclpy.init(args=args)

   pclsub = PCLSubscriber()

   rclpy.spin(pclsub)


   # Destroy the node explicitly
   # (optional - otherwise it will be done automatically
   # when the garbage collector destroys the node object)
   pclsub.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()