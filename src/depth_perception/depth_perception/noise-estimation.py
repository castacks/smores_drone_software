import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from cv_bridge import CvBridge
import numpy as np
import cv2
from tqdm import tqdm
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

def prep(img, map_x, map_y):
    img_rect = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)
    # p1, p99 = np.percentile(img_rect), [10, 90])
    # img_left_clipped = np.clip(left_rect, p1, p99)
    # img_right_clipped = np.clip(right_rect, p1, p99)
    # img_left_u8 = cv2.normalize(
    #     img_left_clipped, None, alpha=0, beta=255,
    #     norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
    # )
    # img_right_u8 = cv2.normalize(
    #     img_right_clipped, None, alpha=0, beta=255,
    #     norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
    # )
    return img_rect

def extract_thermal_images(bag_path: str):
    # Initialize ROS2
    rclpy.init()
    
    # Create Sequential Reader
    reader = rosbag2_py.SequentialReader()
    
    # Configure storage and conversion options
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='sqlite3'  # or 'sqlite3' depending on your bag format
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    # Open the bag file
    reader.open(storage_options, converter_options)
    
    # Create CV Bridge for image conversion
    bridge = CvBridge()
    
    # Initialize lists to store images
    left_images = []
    right_images = []
    
    # Get topic metadata
    topics = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topics}
    
    try:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            
            # Process only thermal topics
            if topic in ['/thermal_left/image', '/thermal_right/image']:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                
                # Convert ROS Image to numpy array
                try:
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                
                    # Append to appropriate list
                    if topic == '/thermal_left/image':
                        # print("Appended Left")
                        left_images.append(cv_image)
                        # print("Showing")
                        # cv2.imshow('img', cv_image)
                        # cv2.waitKey(0)
                    else:
                        # print("Appended Right")
                        right_images.append(cv_image)
                except TypeError:
                    pass
    finally:
        del reader  # Ensure proper cleanup
        rclpy.shutdown()
    
    return left_images, right_images

# Usage example:
left_images, right_images = extract_thermal_images('/external/data/airlab1_ros2/')
kernel = np.ones((5, 5), np.float32)
kernel[2, 2] = 0
kernel = kernel / np.sum(kernel)
left_surrounding_minus_themself = []
mean_noise_left = np.zeros_like(left_images[0])
images_seen_left = 0
for left_img in tqdm(right_images):
    rect = prep(left_img, right_map_x, right_map_y)
    surrounding_means = cv2.filter2D(rect, ddepth=-1, kernel=kernel)
    # surrounding_means_norm = cv2.normalize(
    #     surrounding_means, None, alpha=0, beta=255,
    #     norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
    # )
    # cv2.imshow('surrounding_means', surrounding_means_norm)
    noise = surrounding_means - rect
    # noise_mean = np.mean(noise)
    # noise_viz = noise - noise_mean
    # noise_viz = 128 + (128 * ( noise_viz/ (np.max(noise_viz) - np.min(noise_viz))))
    # cv2.imshow('noize_viz', noise_viz.astype(np.uint8))
    # noise_norm = cv2.normalize(
    #     noise, None, alpha=0, beta=255,
    #     norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
    # )
    mean_noise_left = ((mean_noise_left * images_seen_left) + noise) / (images_seen_left + 1)
    images_seen_left = images_seen_left + 1
    # noise_mean_mean = np.mean(mean_noise_left)
    # mean_noise_viz = mean_noise_left - noise_mean_mean
    # mean_noise_viz = 128 + (128 * ( mean_noise_viz/ (np.max(mean_noise_viz) - np.min(mean_noise_viz))))
    # mean_noise_norm = cv2.normalize(
    #     mean_noise_left_viz, None, alpha=0, beta=255,
    #     norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
    # )
    # cv2.imshow('noise', noise)
    # cv2.imshow('mean noise', mean_noise_viz.astype(np.uint8))
    # cv2.waitKey(1)
with open('right.npy', 'wb') as f:
    np.save(f, mean_noise_left)