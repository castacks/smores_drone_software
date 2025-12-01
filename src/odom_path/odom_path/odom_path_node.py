#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomPathNode(Node):
    """
    Simple node that subscribes to odometry messages and publishes a path.
    """

    def __init__(self):
        super().__init__('odom_path_node')

        # Declare parameters for remappable topics
        self.declare_parameter('odom_topic', '/mavros/odometry/out')
        self.declare_parameter('path_topic', 'odom_path')
        self.declare_parameter('max_path_length', 1000)

        # Get parameters
        odom_topic = self.get_parameter('odom_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.max_path_length = self.get_parameter('max_path_length').value

        # Initialize path message
        self.path = Path()

        # Create subscription
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Create publisher
        self.path_pub = self.create_publisher(
            Path,
            path_topic,
            10
        )

        self.odom_count=0
        self.get_logger().info(f'Subscribed to: {odom_topic}')
        self.get_logger().info(f'Publishing path to: {path_topic}')
        self.get_logger().info(f'Max path length: {self.max_path_length}')

    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry messages. Appends pose to path and publishes.
        """
        self.odom_count+=1
        # Create PoseStamped from odometry
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        # Update path
        if self.odom_count%10==0:
            self.path.header = msg.header
            self.path.poses.append(pose_stamped)

        # Limit path length
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)

        # Publish path
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPathNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
