import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
# from cam_interfaces.msg import MoGEOutput, StereoPair

# sys.path.append("/external/smores_drone_software/include")
# sys.path.append("/external/smores_drone_software/include/MoGe")
# from moge.model.v1 import MoGeModel
from cv_bridge import CvBridge
import cv2
import numpy as np

# [404.9842534577856, 405.0992911907136, 313.1521147858522, 237.73982476898445]
K1 = np.array([[404.9842534577856, 0, 313.1521147858522],
               [0, 405.0992911907136, 237.73982476898445],
               [0, 0, 1]])

# Right camera intrinsics
# [405.14461975696844, 405.25932189028606, 311.42449426256036, 242.08074351877391]
K2 = np.array([[405.14461975696844, 0, 311.42449426256036],
               [0, 405.25932189028606, 242.08074351877391],
               [0, 0, 1]])

# Distortion coefficients (k1, k2, p1, p2, k3)
# [-0.3418843277284295, 0.09554844659447544, 0.0006766728551819399, 0.00013250437150091342]
D1 = np.array([-0.3418843277284295, 0.09554844659447544, 0.0006766728551819399, 0.00013250437150091342, 0])

D2 = np.array([-0.34589878843358174, 0.09955873180627223, 0.00035223176803024823, -0.0007594805806849117, 0])

# Extrinsic parameters
R = np.eye(3)  # Rotation between cameras
T = np.array([-0.2436616774623335, 0.0, 0.0])  # Translation vector
image_size = (640, 512)

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
    newImageSize=image_size
)
left_map_x, left_map_y = cv2.initUndistortRectifyMap(
    K1, D1, R1, P1, image_size, cv2.CV_32FC1)
right_map_x, right_map_y = cv2.initUndistortRectifyMap(
    K2, D2, R2, P2, image_size, cv2.CV_32FC1)

num_disparities = 16 * 3  # Must be divisible by 16
block_size = 21  # Must be odd
stereo = cv2.StereoBM_create(numDisparities=num_disparities, blockSize=block_size)
min_disparity = stereo.getMinDisparity()
max_disparity = min_disparity + stereo.getNumDisparities() - 1
raw_min_valid = min_disparity * 16
raw_max_valid = max_disparity * 16
clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))


class SyncDepthPair(Node):
    def __init__(self):
        super().__init__('sync_depth_pair')

        left_sub = message_filters.Subscriber(self, Image, "/thermal_left/image")
        right_sub = message_filters.Subscriber(self, Image, "/thermal_right/image")

        ats = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.pub_synced_pair)

        # self.pair_publisher = self.create_publisher(StereoPair, "thermal/moge/depthpairs", 10)
        self.disparity_publisher = self.create_publisher(Image, "/thermal/disparity", 10)
        self.left_publisher = self.create_publisher(Image, "/thermal/left_cleaned", 10)
        self.right_publisher = self.create_publisher(Image, "/thermal/right_cleaned", 10)
        self.bridge = CvBridge()

        # Load the model using downloaded file

        # if torch.cuda.is_available():
        #     self.device = torch.device("cuda")
        # else:
        #     self.device = torch.device("cpu")
        # self.model = MoGeModel.from_pretrained(
        #     "/external/smores_drone_software/include/MoGe/moge/model/weights/model.pt").to(self.device)

    def pub_synced_pair(self, msg_left, msg_right):
        self.get_logger().info(f'Received synchronized messages: X={msg_left.header.stamp}, Y={msg_right.header.stamp}')
        img_left = self.bridge.imgmsg_to_cv2(msg_left, msg_left.encoding)
        img_right = self.bridge.imgmsg_to_cv2(msg_right, msg_right.encoding)
        left_rect = cv2.remap(img_left, left_map_x, left_map_y, cv2.INTER_LINEAR)
        right_rect = cv2.remap(img_right, right_map_x, right_map_y, cv2.INTER_LINEAR)
        p1, p99 = np.percentile(np.asarray([left_rect, right_rect]), [10, 90])
        img_left_clipped = np.clip(left_rect, p1, p99)
        img_right_clipped = np.clip(right_rect, p1, p99)
        img_left_u8 = cv2.normalize(
            img_left_clipped, None, alpha=0, beta=255,
            norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        # img_left_u8 = cv2.GaussianBlur(img_left_u8, (5, 5), 1)
        # img_left_u8 = cv2.bilateralFilter(img_left_u8, 9, 150, 10)
        # img_left_u8 = clahe.apply(img_left_u8)
        img_right_u8 = cv2.normalize(
            img_right_clipped, None, alpha=0, beta=255,
            norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        # img_right_u8 = cv2.GaussianBlur(img_right_u8, (5, 5), 1)
        # img_right_u8 = cv2.bilateralFilter(img_right_u8, 9, 150, 10)
        # img_right_u8 = clahe.apply(img_right_u8)
        disparity = stereo.compute(img_left_u8, img_right_u8)
        disparity_norm = 255 * (disparity - raw_min_valid) / (raw_max_valid - raw_min_valid)
        disparity_norm = np.clip(disparity_norm, 0, 255)
        disparity_normalized_u8 = disparity_norm.astype(np.uint8)
        # disparity_normalized = cv2.normalize(
        #     disparity, None, alpha=0, beta=255,
        #     norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
        # )
        # actual_disparity_map = disparity.astype(np.float32) / 16.0

        self.disparity_publisher.publish(self.bridge.cv2_to_imgmsg(disparity_normalized_u8))
        self.left_publisher.publish(self.bridge.cv2_to_imgmsg(img_left_u8))
        self.right_publisher.publish(self.bridge.cv2_to_imgmsg(img_right_u8))
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
