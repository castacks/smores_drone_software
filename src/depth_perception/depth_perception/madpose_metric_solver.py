import rclpy
import os, sys

ws_dir = os.getenv("ROS_WS_DIR", "/external/smores_drone_software")

sys.path.append("/opt/conda/lib/python3.10/site-packages/")
sys.path.append(f"{ws_dir}/include/MoGe")
sys.path.append(f"{ws_dir}/include/ros2_numpy")

import cv2
import numpy as np
import yaml

# ROS imports
from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField
from cam_interfaces.msg import MoGEOutput
import message_filters
import ros2_numpy

# import helper method to visualize depth
from moge.utils.vis import colorize_depth

# madpose imports
import madpose
from madpose.utils import bougnoux_numpy, compute_pose_error, get_depths
import open3d as o3d

class MADPoseSolver(Node):

    def __init__(self):
        super().__init__('madpose_solver')

        self.bridge = CvBridge()

        # load intrinscis
        self.declare_parameter('left_cam_intrinsics_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter('right_cam_intrinsics_file', rclpy.Parameter.Type.STRING)
        self.load_intrinsics_both()

        # Thresholds for reprojection and epipolar errors
        reproj_pix_thres = 16.0
        epipolar_pix_thres = 1.0

        # Weight for epipolar error
        epipolar_weight = 1.0

        # Ransac Options and Estimator Configs
        self.options = madpose.HybridLORansacOptions()
        self.options.min_num_iterations = 100
        self.options.max_num_iterations = 1000
        self.options.final_least_squares = True
        self.options.threshold_multiplier = 5.0
        self.options.num_lo_steps = 4
        self.options.squared_inlier_thresholds = [reproj_pix_thres**2, epipolar_pix_thres**2]
        self.options.data_type_weights = [1.0, epipolar_weight]
        self.options.random_seed = 0

        self.est_config = madpose.EstimatorConfig()
        self.est_config.min_depth_constraint = True
        self.est_config.use_shift = True
        self.est_config.ceres_num_threads = 16

        # get synced msg
        left_sub = message_filters.Subscriber(self, MoGEOutput, "thermal_left/moge")
        right_sub = message_filters.Subscriber(self, MoGEOutput, "thermal_right/moge")

        ats = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.solve_metric)

        self.metricdepthmap0_publisher = self.create_publisher(Image, "thermal/madpose/depthmap_left", 10)
        self.metricdepthmap1_publisher = self.create_publisher(Image, "thermal/madpose/depthmap_right", 10)
        self.ptcl0_publisher = self.create_publisher(PointCloud2, "thermal/madpose/pointcloud_left", 10)
        self.ptcl1_publisher = self.create_publisher(PointCloud2, "thermal/madpose/pointcloud_right", 10)

        self.i = 0

    def test_synced_pair(self, msg_left, msg_right):
        self.get_logger().info(f'{self.i}: Received synchronized messages left ={msg_left.header.stamp}, right={msg_right.header.stamp}')
        self.i += 1

    def load_intrinsics_both(self):
        self.K0 = self._load_intrinsics("left")
        self.K1 = self._load_intrinsics("right")

    def _load_intrinsics(self, cam):
        intrinsics_file = self.get_parameter(f'{cam}_cam_intrinsics_file').value
        with open(intrinsics_file, 'r') as file:
            cal = yaml.safe_load(file)

        intrinsics = cal["cam0"]["intrinsics"]

        fx = intrinsics[0]
        fy = intrinsics[1]
        cx = intrinsics[2]
        cy = intrinsics[3]

        cameraIntrinsics = np.array([[fx,  0, cx],
                                [0, fy, cy],
                                [0,  0,  1]])
        
        return cameraIntrinsics
    
    def get_matched_keypoints(self, image0, image1):
        # Initialize SIFT detector
        sift = cv2.SIFT_create()

        # Detect keypoints and compute descriptors
        kp0, des0 = sift.detectAndCompute(image0, None)
        kp1, des1 = sift.detectAndCompute(image1, None)

        # Initialize BFMatcher
        bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

        # Match descriptors
        matches = bf.match(des0, des1)

        # Sort matches by distance
        matches = sorted(matches, key=lambda x: x.distance)

        # Extract matched keypoints
        mkpts0 = np.array([kp0[m.queryIdx].pt for m in matches])
        mkpts1 = np.array([kp1[m.trainIdx].pt for m in matches])

        return mkpts0, mkpts1


    def solve_metric(self, msg_left, msg_right):
        # Read the image pair
        image0 = self.bridge.imgmsg_to_cv2(msg_left.preproc, msg_left.preproc.encoding)
        image1 = self.bridge.imgmsg_to_cv2(msg_right.preproc, msg_right.preproc.encoding)

        # Run keypoint detector (SIFT)
        mkpts0, mkpts1 = self.get_matched_keypoints(image0, image1)

        # read depth map
        depth_map0 = np.array(msg_left.depth).reshape((msg_left.preproc.height, msg_left.preproc.width))
        depth_map1 = np.array(msg_right.depth).reshape((msg_right.preproc.height, msg_right.preproc.width))

        # Query the depth priors of the keypoints
        depth0 = get_depths(image0, depth_map0, mkpts0)
        depth1 = get_depths(image1, depth_map1, mkpts1)

        pose, stats = madpose.HybridEstimatePoseScaleOffset(
            mkpts0,
            mkpts1,
            depth0,
            depth1,
            [depth_map0.min(), depth_map1.min()],
            self.K0,
            self.K1,
            self.options,
            self.est_config,
        )
        # rotation and translation of the estimated pose
        R_est, t_est = pose.R(), pose.t()
        # scale and offsets of the affine corrected depth maps
        s_est, o0_est, o1_est = pose.scale, pose.offset0, pose.offset1

        depth_map0_world = s_est * (depth_map0 + o0_est)
        depth_map1_world = s_est * (depth_map1 + o1_est)

        processed_depthmap0 = cv2.cvtColor(colorize_depth(depth_map0_world), cv2.COLOR_RGB2BGR) # (512, 640, 3)
        processed_depthmap1 = cv2.cvtColor(colorize_depth(depth_map1_world), cv2.COLOR_RGB2BGR)
        # self.get_logger().info(f"{processed_depthmap0.shape}")
        #cv2.imwrite(f"data/test/dm/{self.i}_depth_map_0.png", processed_depthmap0)
        #cv2.imwrite(f"data/test/dm/{self.i}_depth_map_1.png", processed_depthmap1)
        self.metricdepthmap0_publisher.publish(self.bridge.cv2_to_imgmsg(processed_depthmap0))
        self.metricdepthmap1_publisher.publish(self.bridge.cv2_to_imgmsg(processed_depthmap1))

        #cv2.imwrite("data/preprocessed_test/depth_map_0.png", depth_map0_world)
        #cv2.imwrite("data/preprocessed_test/depth_map_1.png", depth_map1_world)
        #self.visualize_depth(depth_map0_world, "depth_map_0_coloured.png")
        #self.visualize_depth(depth_map1_world, "depth_map_1_coloured.png")

        fx0 = self.K0[0, 0]
        fy0 = self.K0[1, 1]
        fx1 = self.K1[0, 0]
        fy1 = self.K1[1, 1]

        cx0 = self.K0[0, 2]
        cy0 = self.K0[1, 2]
        cx1 = self.K1[0, 2]
        cy1 = self.K1[1, 2]
        
        point_cloud0, colors0 = self.depth_to_point_cloud(depth_map0_world, image0, cx0, cy0, fx0, fy0)
        point_cloud1, colors1 = self.depth_to_point_cloud(depth_map1_world, image1, cx1, cy1, fx1, fy1)

        #np.savez(f"data/test/npy/{self.i}_npy.npy", point_cloud0, colors0)

        # Save the point clouds in PLY format
        self.save_point_cloud(point_cloud0, colors0, f"data/test/pcltx/{self.i}_point_cloud_0.ply")
        self.save_point_cloud(point_cloud1, colors1, f"data/test/pcltx/{self.i}_point_cloud_1.ply")
        self.i += 1

        pc20 = self.create_pc2_msg("thermal_left/optical_frame", point_cloud0, colors0)
        pc21 = self.create_pc2_msg("thermal_right/optical_frame", point_cloud1, colors1)
        self.ptcl0_publisher.publish(pc20)
        self.ptcl1_publisher.publish(pc21)

    def visualize_depth(self, depth_map, filename):
        # Normalize the depth map for visualization
        depth_min = np.min(depth_map)
        depth_max = np.max(depth_map)
        depth_normalized = (depth_map - depth_min) / (depth_max - depth_min)

        # Apply a color map to the normalized depth map
        depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)

        # Display the depth map
        cv2.imwrite(f"data/preprocessed_test/{filename}", depth_colored)

    def depth_to_point_cloud(self, depth_map, image, cx, cy, fx, fy):
        # expect image to be grayscale depth map
        h, w = depth_map.shape[:2]
        i, j = np.meshgrid(np.arange(w), np.arange(h), indexing='xy')
        z = depth_map
        x = (i - cx) * z / fx
        y = (j - cy) * z / fy
        point_cloud = np.stack((x, y, z), axis=-1) # (512, 640, 3)

        # Get the colors from the image
        colors = image[j, i] / np.max(image) * 255
        colors = np.stack((colors, colors, colors), axis=-1).astype(np.uint32)
        return point_cloud, colors
    
    def create_pc2_msg(self, frame_id: str, points: np.ndarray, colors: np.ndarray=None):
        # points and colors should be (h, w, 3) or (n, 3) where n = h*w
        points = points.reshape(-1, 3)
        colors = colors.reshape(-1, 3)

        merged_colors = self.merge_rgb(colors)

        dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
            ]

        data = np.empty(points.shape[0], dtype=dtype)
        data['x'] = points[:, 0]
        data['y'] = points[:, 1]
        data['z'] = points[:, 2]
        data['rgb'] = merged_colors
        
        msg = ros2_numpy.msgify(PointCloud2, data, frame_id=frame_id)
        return msg

    def merge_rgb(self, colors: np.ndarray):
        r = np.asarray(colors[:, 0], dtype=np.uint32)
        g = np.asarray(colors[:, 1], dtype=np.uint32)
        b = np.asarray(colors[:, 2], dtype=np.uint32)
        rgb_arr = np.array((r << 16) | (g << 8) | (b << 0), dtype=np.uint32)

        return rgb_arr

    def save_point_cloud(self, points, colors, filename):
        # Convert the numpy array to an Open3D point cloud
        points = points.reshape(-1, 3)
        colors = colors.reshape(-1, 3)

        # Convert the numpy array to an Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Save the point cloud to a file
        o3d.io.write_point_cloud(filename, pcd)

        self.get_logger().info(f"Saved PLY: {filename}")

        # o3d.visualization.draw_plotly([pcd])

def main(args=None):
   rclpy.init(args=args)

   madpose_solver = MADPoseSolver()

   rclpy.spin(madpose_solver)


   # Destroy the node explicitly
   # (optional - otherwise it will be done automatically
   # when the garbage collector destroys the node object)
   madpose_solver.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()