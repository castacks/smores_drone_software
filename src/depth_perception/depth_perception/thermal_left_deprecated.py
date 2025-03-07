import rclpy
import os, sys
sys.path.append("/opt/conda/lib/python3.10/site-packages/")
import cv2


from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

import numpy as np
from rclpy.executors import MultiThreadedExecutor


# from MoGe.moge.model import MoGeModel NOTE: No module named MoGe


bridge = CvBridge()


class ThermalPubSub(Node):


    def __init__(self):
        super().__init__('preprocess_thermal_pub_sub_left')


        # TODO: Update to sync with the cameras directly
        self.subscription = self.create_subscription(
            Image,
            'thermal_left/image',
            self.publish_preprocessed_image,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 0

        self.publisher = self.create_publisher(
            Image,
            'thermal_left_preprocessed/image',
            10
        )


    def preprocess_image(self, image: Image):
        """
        Description: Preprocessed raw thermal image by applying outlier filtering, histogram equalization, bilateral filtering (edge-aware smoothing)\n
        Topic: thermal_left/image\n
        Input: image -> sensor_msgs.msg.Image\n
        Output: cv_img -> cv2.CV_16UC1\n
        """
        # Image encoding is 16UC1
        cv_img = bridge.imgmsg_to_cv2(image, image.encoding)
        # Apply histogram filtering, CLAHE, Bilateral Filtering
        cv_img = self.filter_outliers(cv_img)
        cv_img = self.apply_clahe(cv_img)
        cv_img = self.bilateral_filtering(cv_img, d=50, sigmaColor=0, sigmaSpace=50)
            
        return cv_img


    def publish_preprocessed_image(self, img: Image):
        """
        Description: Publishes preprocessed image to designated topic\n
        Input: image -> sensor_msgs.msg.Image\n
        Output: cv_img -> sensor_msgs.msg.Image\n
        Topic: thermal_preprocessed_left/image
        """
        cv_img = self.preprocess_image(img)
        img = bridge.cv2_to_imgmsg(cv_img)
        self.publisher.publish(img)
        cv2.imwrite(f'/data/image_stream/preprocessed_left_image_{self.i}.png')

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


   preprocess_thermal_pub_sub = ThermalPubSub()
  
   executor = MultiThreadedExecutor()
   executor.add_node(preprocess_thermal_pub_sub)


   try:
       executor.spin()
   finally:
       preprocess_thermal_pub_sub.destroy_node()
       rclpy.shutdown()


   # rclpy.spin(preprocess_thermal_pub_sub)


   # # Destroy the node explicitly
   # # (optional - otherwise it will be done automatically
   # # when the garbage collector destroys the node object)
   # preprocess_thermal_pub_sub.destroy_node()
   # rclpy.shutdown()




if __name__ == '__main__':
   main()
