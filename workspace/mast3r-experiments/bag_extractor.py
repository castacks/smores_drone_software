import rclpy.serialization
import rosbag2_py as bag
from cv_bridge import CvBridge
import numpy as np
import cv2
import rclpy
import sensor_msgs
import sensor_msgs.msg
import cv2

left_cam_matrix = np.asarray(
        [
            [405.57512495273403, 0, 313.2365778080362],
            [0, 405.5592192044737, 237.85962939282086],
            [0, 0, 1],
        ]
    )
left_distortion_coeffs = np.asarray(
        [
            -0.3448173534502492,
            0.09834339989635991,
            0.0006913356388736054,
            -0.0001326767741732132,
        ]
    )


def preprocess(source, fraction):
    print(source.dtype)
    total_pixels = np.size(source)
    sorted = np.sort(source.flatten())
    print(sorted.shape)
    low_percentile = sorted[int(total_pixels * fraction)]
    high_percentile = sorted[int(total_pixels * (1 - fraction))]
    print(low_percentile, high_percentile)
    output = source.astype(np.float64)
    output = np.minimum(np.maximum(output, low_percentile), high_percentile)
    output = (output - low_percentile) / (high_percentile - low_percentile)
    output = output * 255
    return output.astype(np.uint8)


def undistort(im, cam_matrix, distortion_coeffs):
    undistorted = cv2.undistort(
        im,
        cam_matrix,
        distortion_coeffs
    )
    return undistorted


reader = bag.SequentialReader()
storage_options = bag.StorageOptions(
    uri="/storage/ros2bags/room/", storage_id="sqlite3"
)
converter_options = bag.ConverterOptions("", "")
reader.open(storage_options, converter_options)

bridge = CvBridge()

i = 0
while reader.has_next():
    msg = reader.read_next()
    if msg[0] == "/thermal_left/image":
        print(i)
        if i % 10 == 0:
            msg_raw = rclpy.serialization.deserialize_message(msg[1], sensor_msgs.msg.Image)
            # print(msg)
            im = bridge.imgmsg_to_cv2(msg_raw, "passthrough")
            prepped = preprocess(im, 0.01)
            undistorted = undistort(prepped, left_cam_matrix, left_distortion_coeffs)
            cv2.imwrite(f"/workspace/mast3r-experiments/imgs/im{i}.png", undistorted)
        i += 1
