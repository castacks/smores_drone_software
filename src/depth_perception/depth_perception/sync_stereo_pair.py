import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from cam_interfaces.msg import MoGEOutput, StereoPair

class SyncDepthPair(Node):
   def __init__(self):
      super().__init__('sync_depth_pair')

      left_sub = message_filters.Subscriber(self, MoGEOutput, "thermal_left/moge")
      right_sub = message_filters.Subscriber(self, MoGEOutput, "thermal_right/moge")

      ats = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.1)
      ats.registerCallback(self.pub_synced_pair)

      self.pair_publisher = self.create_publisher(StereoPair, "thermal/moge/depthpairs", 10)

   def pub_synced_pair(self, msg_left, msg_right):
      self.get_logger().info(f'Received synchronized messages: X={msg_left.header.stamp}, Y={msg_right.header.stamp}')

      # msg = StereoPair()
      # msg.preproc_left = msg_left.preproc
      # msg.preproc_right = msg_right.preproc
      # msg.depth_left = msg_left.depth
      # msg.depth_right = msg_right.depth

      # self.pair_publisher.publish(msg)

def main(args=None):
   rclpy.init(args=args)

   sync_pair = SyncDepthPair()
   
   rclpy.spin(sync_pair)


   # Destroy the node explicitly
   # (optional - otherwise it will be done automatically
   # when the garbage collector destroys the node object)
   sync_pair.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()
