import rclpy
from pathlib import Path
import numpy as np
import os, sys

ws_dir = os.getenv("ROS_WS_DIR", "/external/smores_drone_software")
# sys.path.append("/opt/conda/lib/python3.10/site-packages/")
sys.path.append(f"{ws_dir}/include")
sys.path.append(f"{ws_dir}/include/MoGe")
from moge.model.v1 import MoGeModel
from moge.utils.vis import colorize_depth
from cam_interfaces.msg import TSMoGEOutput
import message_filters

# colcon build --symlink-install && source install/setup.bash && ros2 launch depth_perception launch.py
# colcon build --packages-select depth_perception && source install/setup.bash && ros2 launch depth_perception launch.py

import cv2
import torch

# ROS imports
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class MogeInference(Node):

    def __init__(self):
        super().__init__('moge_infer_depth')

        self.bridge = CvBridge()

        # Load the model using downloaded file

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
            self.get_logger().info(f"MogeInferenceMono.py: Loaded GPU successfully")

        else:
            self.device = torch.device("cpu")
            self.get_logger().warn(f"MogeInferenceMono.py: Loaded from the CPU. GPU unavailable")
             
        self.model = MoGeModel.from_pretrained(f"{ws_dir}/include/MoGe/moge/model/weights/model.pt").to(self.device)

        # registering necessary publishers and subscribers
        left_thermal_subscriber = message_filters.Subscriber(self, Image, "/thermal_left/preprocd_image")
        right_thermal_subscriber = message_filters.Subscriber(self, Image, "/thermal_right/preprocd_image")
        
        self.left_depthmap_publisher = self.create_publisher(Image,  "thermal_left/moge/depthmap", 10) #
        self.right_depthmap_publisher = self.create_publisher(Image,  "thermal_right/moge/depthmap", 10) #
        
        self.moge_publisher = self.create_publisher(TSMoGEOutput, "thermal/moge", 10)  # Has image + depth map
        
        # Soft time sync between incoming images
        queue_size = 10
        max_delay_s =  0.1
        ats = message_filters.ApproximateTimeSynchronizer( (left_thermal_subscriber, right_thermal_subscriber),
                                                                        queue_size, max_delay_s)
        ats.registerCallback(self.infer_depth)
        
        self.get_logger().info(f"MogeInferenceMono.py: Initialized MogeInference node successfully")
        
        self.count = 0
        

    def infer_depth(self, left_image: Image, right_image:Image):
        """
        Description: Preprocessed raw thermal image by applying outlier filtering, histogram equalization, bilateral filtering (edge-aware smoothing)\n
        Args:
            left_image: sensor_msgs.msg.Image - Thermal image from the left camera of size 640x512 output from the preproc node
            right_image: sensor_msgs.msg.Image - Thermal image from the right camera of size 640x512 output from the preproc node

        Returns:
            Publishes a message of type TSMoGEOutput with the left, right preprocessed images and moge output depthmaps
            Method does not return anything
        """

        cv2_left  = self.bridge.imgmsg_to_cv2(left_image, left_image.encoding)
        cv2_right = self.bridge.imgmsg_to_cv2(right_image, right_image.encoding)

        cv2_left_rgb  = cv2.cvtColor(cv2_left, cv2.COLOR_BGR2RGB)
        cv2_right_rgb = cv2.cvtColor(cv2_right, cv2.COLOR_BGR2RGB)
        
        store_image_path = f"/workspace/smores_drone_software/data/airlab1_images"
        cv2.imwrite(f"{store_image_path}/left_{self.count}.png", cv2_left_rgb)
        cv2.imwrite(f"{store_image_path}/right_{self.count}.png", cv2_right_rgb)
        
        print(f"{self.count} images saved")
        self.count += 1
        
        
        # height, width = cv2_left_rgb.shape[:2]
        # new_width = int(width/2)
        # height, width = min(new_width, int(new_width * height / width)), min(new_width, int(new_width * width / height))
        # cv2_left_rgb = cv2.resize(cv2_left_rgb, (width, height), cv2.INTER_AREA)
        # cv2_right_rgb = cv2.resize(cv2_right_rgb, (width, height), cv2.INTER_AREA)
        
        cv2_left_tensor = torch.tensor(cv2_left_rgb / 255, dtype=torch.float32, device=self.device).permute(2, 0, 1)   #(channels, length, width)
        cv2_right_tensor = torch.tensor(cv2_right_rgb / 255, dtype=torch.float32, device=self.device).permute(2, 0, 1) #(channels, length, width)
        
        input_tensor = torch.stack([cv2_left_tensor, cv2_right_tensor])
        
        self.get_logger().debug(f"MogeInferenceMono.py: About to run model")
        
        output = self.model.infer(input_tensor, fov_x=95)
        depth = output["depth"].cpu().numpy()
        #points = output["points"].cpu().numpy()
        #intrinsics = output["intrinsics"].cpu().numpy()
        
        self.get_logger().debug(f"MogeInferenceMono.py: Model inference complete")
        
        processed_depthmaps = np.zeros([2, depth.shape[1], depth.shape[2], 3])
        self.get_logger().debug(f"MogeInferenceMono.py: Depth of shape {depth.shape}")
 
        processed_depthmaps[0, :, :, :] = cv2.cvtColor(colorize_depth(depth[0]), cv2.COLOR_RGB2BGR)
        processed_depthmaps[1, :, :, :] = cv2.cvtColor(colorize_depth(depth[1]), cv2.COLOR_RGB2BGR)
        
        self.publish_both_depthmaps(processed_depthmaps)

        msg = TSMoGEOutput()
        msg.header = left_image.header
        msg.left_image = left_image
        msg.right_image = right_image
        msg.left_depth = depth[0].flatten().tolist()
        msg.right_depth = depth[1].flatten().tolist()
        self.moge_publisher.publish(msg)

    def publish_both_depthmaps(self, image):
        """
       Description: Publishes preprocessed image to designated topic
       Input: image -> sensor_msgs.msg.Image
       Output: cv_img -> sensor_msgs.msg.Image
       Topic: thermal_preprocessed_left/image
       """
        left_img = self.bridge.cv2_to_imgmsg(image[0])
        self.get_logger().debug(f"DepthMap size is {image[0].shape}")
        right_img = self.bridge.cv2_to_imgmsg(image[1])
        
        self.left_depthmap_publisher.publish(left_img)
        self.right_depthmap_publisher.publish(right_img)


def main(args=None):
   rclpy.init(args=args)

   moge_inference = MogeInference()

   rclpy.spin(moge_inference)

   moge_inference.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()
