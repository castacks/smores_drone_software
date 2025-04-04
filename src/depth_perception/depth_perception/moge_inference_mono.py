import rclpy
from pathlib import Path
import numpy as np
import os, sys
import pdb
# sys.path.append("/opt/conda/lib/python3.10/site-packages/")
ws_dir = os.getenv("ROS_WS_DIR", "/external/smores_drone_software")
sys.path.append(f"{ws_dir}/include")
sys.path.append(f"{ws_dir}/include/MoGe")
from moge.model.v1 import MoGeModel
from moge.utils.vis import colorize_depth
from cam_interfaces.msg import MoGEOutput

# colcon build --symlink-install && source install/setup.bash && ros2 launch depth_perception launch.py
# colcon build --packages-select depth_perception && source install/setup.bash && ros2 launch depth_perception launch.py

import cv2
import torch

# ROS imports
from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, PointCloud
from rclpy.executors import MultiThreadedExecutor

class MogeInference(Node):

    def __init__(self):
        super().__init__('moge_infer_depth')

        self.bridge = CvBridge()

        # Load the model using downloaded file

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")
        self.model = MoGeModel.from_pretrained(f"{ws_dir}/include/MoGe/moge/model/weights/model.pt").to(self.device)


        # TODO: Update to sync with the cameras directly
        self.subscription = self.create_subscription(
            Image,
            "thermal/image",
            self.infer_depth,
            1,
        )
        self.subscription  # prevent unused variable warning

        self.depthmap_publisher = self.create_publisher(Image, "thermal/moge/depthmap", 10)
        self.moge_publisher = self.create_publisher(MoGEOutput, "thermal/moge", 10)

        self.i = 0

        self.get_logger().info(f"MogeInferenceMono.py: Initialized MogeInference node successfully")

    def preproc(self, image):
        # Apply histogram filtering, CLAHE, Bilateral Filtering
        cv_img = self.filter_outliers(image)
        cv_img = self.apply_clahe(cv_img)
        cv_img = self.bilateral_filtering(cv_img, d=50, sigmaColor=0, sigmaSpace=50)
        #cv2.imwrite(f"data/moge_ros_test_preproc/img_{self.i}.tiff", cv_img)
        self.publish_preproc(cv_img)

        return cv_img

    def infer_depth(self, image: Image):
        """
        Description: Preprocessed raw thermal image by applying outlier filtering, histogram equalization, bilateral filtering (edge-aware smoothing)\n
        Topic: thermal_left/image
        Input: image -> sensor_msgs.msg.Image
        Output: cv_img -> cv2.CV_16UC1
        """

        """ TODO
        See how many GPUS this process has access to => set env variable ~VISIBLE_GPUs~=0,1,2,3
        How to parallelize a model run across GPUs
        
        See how multithreading is implemented
        See how queues are handled
        See how new subscriber calls are handled when one is still running WITH Multithreading and WITHOUT
        """

        cv2_img = self.bridge.imgmsg_to_cv2(image, image.encoding)
        #cv2_img = self.preproc(cv2_img)

        input_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)

        # convert to tensor
        input_tensor = torch.tensor(input_img / 255, dtype=torch.float32, device=self.device).permute(2, 0, 1)
        
        output = self.model.infer(input_tensor, fov_x=95)
        depth = output["depth"].cpu().numpy()
        points = output["points"].cpu().numpy()
        intrinsics = output["intrinsics"].cpu().numpy()
        mask = output["mask"].cpu().numpy()
        
        processed_depthmap = cv2.cvtColor(colorize_depth(depth), cv2.COLOR_RGB2BGR)
        #cv2.imwrite(f"data/moge_ros_test/depth_vis_{self.i}.png", processed_depthmap)
        self.publish_depthmap(processed_depthmap)

        msg = MoGEOutput()
        msg.header = image.header
        msg.preproc = image
        msg.depth = depth.flatten().tolist()
        self.moge_publisher.publish(msg)

    def publish_preproc(self, image):
        """
       Description: Publishes preprocessed image to designated topic\n
       Input: image -> sensor_msgs.msg.Image\n
       Output: cv_img -> sensor_msgs.msg.Image\n
       Topic: thermal_preprocessed_left/image
       """
        img = self.bridge.cv2_to_imgmsg(image)
        self.preproc_publisher.publish(img)


    def publish_depthmap(self, image):
        """
       Description: Publishes preprocessed image to designated topic\n
       Input: image -> sensor_msgs.msg.Image\n
       Output: cv_img -> sensor_msgs.msg.Image\n
       Topic: thermal_preprocessed_left/image
       """
        img = self.bridge.cv2_to_imgmsg(image)
        self.depthmap_publisher.publish(img)

    def publish_valid_pcl(self):
        # TODO: Add documentation and possibly pointmap class
        self.pcl_publisher(self.points[self.mask].cpu().numpy())

    def publish_intrinsics(self):
        self.intrinsics_publisher(self.intrinsics.cpu().numpy())

    def filter_outliers(self, img, lower_percentile=1, upper_percentile=99):
        # Filtering outliers above and below 2 certain thresholds. Used to remove extremely bright spots like the sun
        lower_bound = np.percentile(img, lower_percentile)
        upper_bound = np.percentile(img, upper_percentile)
        filtered_img = np.clip(img, lower_bound, upper_bound)
        filtered_img = ((filtered_img - lower_bound) / (upper_bound - lower_bound) * 255).astype(np.uint8)
        return filtered_img

    def apply_clahe(self, img):
        # Applies CLAHE (a specific type of histogram equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return clahe.apply(img)

    def bilateral_filtering(self, img, d=9, sigmaColor=75, sigmaSpace=75):
        # Applies edge aware smoothing, smoothing type is gaussian blur under the hood
        return cv2.bilateralFilter(img, d=d, sigmaColor=sigmaColor, sigmaSpace=sigmaSpace)


def main(args=None):
   rclpy.init(args=args)


   moge_inference = MogeInference()
  
   # executor = MultiThreadedExecutor()
   # executor.add_node(moge_inference)


   # try:
   #     executor.spin()
   # finally:
   #     moge_inference.destroy_node()
   #     rclpy.shutdown()


   rclpy.spin(moge_inference)


   # Destroy the node explicitly
   # (optional - otherwise it will be done automatically
   # when the garbage collector destroys the node object)
   moge_inference.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()
