import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from cv_bridge import CvBridge
import numpy as np
import cv2
from tqdm import tqdm
from parameters import *


R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    cameraMatrix1=K1,
    cameraMatrix2=K2,
    distCoeffs1=D1,
    distCoeffs2=D2,
    R=R,
    T=T,
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


def prep(img, map_x, map_y):
    img_rect = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)
    return img_rect


oneD_kernel = cv2.getGaussianKernel(21, sigma=8)
kernel = oneD_kernel * oneD_kernel.T
kernel[10, 10] = 0
kernel = kernel / np.sum(kernel)


def estimate_noise(bag_path: str):
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id="sqlite3"  # or 'sqlite3' depending on your bag format
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader.open(storage_options, converter_options)
    metadata = reader.get_metadata()
    topic_message_counts = {}

    # Iterate over topic metadata
    for topic_info in metadata.topics_with_message_count:
        topic_message_counts[topic_info.topic_metadata.name] = topic_info.message_count
    left_total = topic_message_counts["/thermal_left/image"]
    right_total = topic_message_counts["/thermal_right/image"]
    left_processed = 0
    right_processed = 0
    progress_bar = tqdm(total=left_total + right_total)

    bridge = CvBridge()

    noise_left = np.zeros((512, 640), np.float128)
    noise_right = np.zeros((512, 640), np.float128)

    topics = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topics}
    start_time = None

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        # print(timestamp)
        if start_time == None:
            start_time = timestamp
        if timestamp - start_time > 260 * 10**9:
            continue
        if topic in ["/thermal_left/image", "/thermal_right/image"]:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if topic == "/thermal_left/image":
                    rect = prep(cv_image, left_map_x, left_map_y).astype(np.float64)
                    surrounding_means = cv2.filter2D(rect, ddepth=-1, kernel=kernel)
                    noise = rect.astype(np.float64) - surrounding_means.astype(
                        np.float64
                    )
                    noise_left = noise_left + noise
                    left_processed = left_processed + 1
                    progress_bar.update(1)
                else:
                    rect = prep(cv_image, right_map_x, right_map_y).astype(np.float64)
                    surrounding_means = cv2.filter2D(rect, ddepth=-1, kernel=kernel)
                    noise = rect.astype(np.float64) - surrounding_means.astype(
                        np.float64
                    )
                    noise_right = noise_right + noise
                    right_processed = right_processed + 1
                    progress_bar.update(1)
            except TypeError:
                print("Skipped!")
    mean_noise_left = noise_left / left_processed
    with open("left.npy", "wb") as f:
        np.save(f, mean_noise_left)
    mean_noise_right = noise_right / right_processed
    with open("right.npy", "wb") as f:
        np.save(f, mean_noise_right)


estimate_noise(
    "/run/media/aayush/OS/LinuxStore/smores/bags/rosbag2_1969_12_31-21_35_04"
)
