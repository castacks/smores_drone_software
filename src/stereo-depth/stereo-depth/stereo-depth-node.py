import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import numpy as np

from parameters import *

R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    cameraMatrix1=K1,
    cameraMatrix2=K2,
    distCoeffs1=D1,
    distCoeffs2=D2,
    R=R,
    T=T,
    flags=cv2.CALIB_ZERO_DISPARITY,
    alpha=0,
    imageSize=image_size,
    newImageSize=image_size,
)
left_map_x, left_map_y = cv2.initUndistortRectifyMap(
    K1, D1, R1, P1, image_size, cv2.CV_32FC1
)
right_map_x, right_map_y = cv2.initUndistortRectifyMap(
    K2, D2, R2, P2, image_size, cv2.CV_32FC1
)

left_noise = np.load(
    "/home/aayush/prj/smores_drone_software/src/stereo-depth/stereo-depth/left.npy"
)
right_noise = np.load(
    "/home/aayush/prj/smores_drone_software/src/stereo-depth/stereo-depth/right.npy"
)
left_noise = left_noise.astype(np.float64)
right_noise = right_noise.astype(np.float64)

window_size = 5
left_matcher = cv2.StereoSGBM_create(
    # minDisparity=0,
    numDisparities=16 * 3,  # max_disp has to be dividable by 16 f. E. HH 192, 256
    blockSize=window_size,
    P1=8 * window_size * window_size,
    # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
    P2=32 * window_size * window_size,
    disp12MaxDiff=1,
    uniquenessRatio=5,
    speckleWindowSize=100,
    speckleRange=2,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    # textureThreshold=100,
)
right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
# FILTER Parameters
lmbda = 70000
sigma = 1.7
visual_multiplier = 6

wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)

min_disparity = left_matcher.getMinDisparity()
max_disparity = min_disparity + left_matcher.getNumDisparities() - 1
raw_min_valid = min_disparity * 16
raw_max_valid = max_disparity * 16


def depth_map(imgL, imgR):
    displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
    # cv2.imshow("displ", displ)
    dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
    displ = np.int16(displ)
    dispr = np.int16(dispr)
    filteredImg = wls_filter.filter(displ, imgL, None, dispr).astype(
        np.float64
    )  # important to put "imgL" here!!!
    disparity_norm = (
        255 * (filteredImg - raw_min_valid) / (raw_max_valid - raw_min_valid)
    )
    disparity_norm = np.clip(disparity_norm, 0, 255)
    disparity_normalized_u8 = disparity_norm.astype(np.uint8)
    disparity_normalized_u8[filteredImg < 0] = 0

    return disparity_normalized_u8


class SyncDepthPair(Node):
    def __init__(self):
        super().__init__("sync_depth_pair")

        left_sub = message_filters.Subscriber(self, Image, "/thermal_left/image")
        right_sub = message_filters.Subscriber(self, Image, "/thermal_right/image")

        ats = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=10, slop=0.005
        )
        ats.registerCallback(self.pub_synced_pair)
        self.disparity_publisher = self.create_publisher(
            Image, "/thermal/disparityv2", 10
        )
        self.left_publisher = self.create_publisher(Image, "/thermal_left/clean", 10)
        self.right_publisher = self.create_publisher(Image, "/thermal_right/clean", 10)
        self.bridge = CvBridge()

    def pub_synced_pair(self, msg_left, msg_right):
        # self.get_logger().info(
        #     f"Received synchronized messages: X={msg_left.header.stamp}, Y={msg_right.header.stamp}"
        # )
        img_left = self.bridge.imgmsg_to_cv2(msg_left, msg_left.encoding)
        # print(img_left.dtype)
        img_right = self.bridge.imgmsg_to_cv2(msg_right, msg_right.encoding)
        left_rect = cv2.remap(img_left, left_map_x, left_map_y, cv2.INTER_LINEAR)
        left_rect = left_rect - left_noise
        right_rect = cv2.remap(img_right, right_map_x, right_map_y, cv2.INTER_LINEAR)
        right_rect = right_rect - right_noise
        # lp1, lp99 = np.percentile(np.asarray(left_rect), [1, 99])
        # rp1, rp99 = np.percentile(np.asarray(right_rect), [1, 99])

        lp1 = 21000
        rp1 = lp1

        lp99 = 21600
        rp99 = lp99

        img_left_clipped = np.clip(left_rect, lp1, lp99)
        img_right_clipped = np.clip(right_rect, rp1, rp99)
        # print(lp1, lp99, rp1, rp99)
        img_left_u8 = cv2.normalize(
            img_left_clipped,
            None,
            alpha=0,
            beta=255,
            norm_type=cv2.NORM_MINMAX,
            dtype=cv2.CV_8U,
        )
        img_right_u8 = cv2.normalize(
            img_right_clipped,
            None,
            alpha=0,
            beta=255,
            norm_type=cv2.NORM_MINMAX,
            dtype=cv2.CV_8U,
        )
        disparity_normalized_u8 = depth_map(img_left_u8, img_right_u8)

        self.disparity_publisher.publish(
            self.bridge.cv2_to_imgmsg(disparity_normalized_u8)
        )
        self.left_publisher.publish(self.bridge.cv2_to_imgmsg(img_left_u8))
        self.right_publisher.publish(self.bridge.cv2_to_imgmsg(img_right_u8))
        # cv2.imshow("left", img_left_u8)
        # cv2.imshow("right", img_right_u8)
        # cv2.imshow("disparity", disparity_normalized_u8)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    sync_pair = SyncDepthPair()
    rclpy.spin(sync_pair)
    sync_pair.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
