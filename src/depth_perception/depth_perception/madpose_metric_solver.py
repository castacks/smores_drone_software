import rclpy
import sys
sys.path.append("/opt/conda/lib/python3.10/site-packages/")
sys.path.append("/workspace/madpose")

import cv2
import numpy as np

# ROS imports
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud
from cam_interfaces.msg import StereoPair

# madpose imports
import madpose
from madpose.utils import bougnoux_numpy, compute_pose_error, get_depths
import open3d as o3d

class MADPoseSolver(Node):

    def __init__(self):
        super().__init__('madpose_solver')

        self.bridge = CvBridge()

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

        self.subscription = self.create_subscription(
            StereoPair, # TODO: custom msg
            "thermal/image", # TODO: topic with depth pairs
            self.solve_metric,
            1,
        )
        self.subscription  # prevent unused variable warning

        self.metricdepthmap_publisher = self.create_publisher(Image, "thermal/madpose/depthmap", 10)
        self.ptcl_publisher = self.create_publisher(PointCloud, "thermal/madpose/pointcloud", 10)

        self.i = 0

    def solve_metric(self, msg):

        # Read the image pair
        image0 = self.bridge.compressed_imgmsg_to_cv2(msg.preproc_left, msg.preproc_left.encoding)
        image1 = self.bridge.compressed_imgmsg_to_cv2(msg.preproc_right, msg.preproc_right.encoding)
        #image0 = cv2.imread("data/preprocessed_test/left.png")
        # image1 = cv2.imread("data/preprocessed_test/right.png")

        # Run keypoint detector (SIFT)
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

        # Load depth maps
        #depth_0_file = "data/preprocessed_test/left_pred_relative.tiff"
        #depth_1_file = "data/preprocessed_test/right_pred_relative.tiff"

        # Load depth images where each pixel value is the X, Y, and Z position
        # depth_map0 = cv2.imread(depth_0_file, cv2.IMREAD_UNCHANGED)
        # depth_map1 = cv2.imread(depth_1_file, cv2.IMREAD_UNCHANGED)
        # depth_map0 = np.array(depth_map0, dtype=np.float32)
        # depth_map1 = np.array(depth_map1, dtype=np.float32)
        depth_map0 = self.bridge.compressed_imgmsg_to_cv2(msg.depth_left, msg.depth_left.encoding)
        depth_map1 = self.bridge.compressed_imgmsg_to_cv2(msg.depth_right, msg.depth_right.encoding)

        # Query the depth priors of the keypoints
        depth0 = get_depths(image0, depth_map0, mkpts0)
        depth1 = get_depths(image1, depth_map1, mkpts1)

        # Compute the principal points
        pp0 = (np.array(image0.shape[:2][::-1]) - 1) / 2
        pp1 = (np.array(image1.shape[:2][::-1]) - 1) / 2
        pp = pp0

        # Run hybrid estimation
        pose, stats = madpose.HybridEstimatePoseScaleOffsetTwoFocal( # why use 2 focal?
            mkpts0,
            mkpts1,
            depth0,
            depth1,
            [depth_map0.min(), depth_map1.min()],
            pp0,
            pp1,
            self.options,
            self.est_config,
        )
        # rotation and translation of the estimated pose
        R_est, t_est = pose.R(), pose.t()
        # scale and offsets of the affine corrected depth maps
        s_est, o0_est, o1_est = pose.scale, pose.offset0, pose.offset1
        # the estimated two focal lengths
        f0_est, f1_est = pose.focal0, pose.focal1

        depth_map0_world = depth_map0 + o0_est
        depth_map1_world = s_est * (depth_map1 + o1_est)

        cv2.imwrite("data/preprocessed_test/depth_map_0.png", depth_map0_world)
        cv2.imwrite("data/preprocessed_test/depth_map_1.png", depth_map1_world)

        cx0, cy0 = pp0
        cx1, cy1 = pp1

        visualize_depth(depth_map0_world, "depth_map_0_coloured.png")
        visualize_depth(depth_map1_world, "depth_map_1_coloured.png")
        point_cloud0 = depth_to_point_cloud(depth_map0_world, image0, cx0, cy0, f0_est)
        point_cloud1 = depth_to_point_cloud(depth_map1_world, image1, cx1, cy1, f1_est)

        # Save the point clouds in PLY format
        save_point_cloud(point_cloud0, "data/preprocessed_test/point_cloud_0.ply")
        save_point_cloud(point_cloud1, "data/preprocessed_test/point_cloud_1.ply")

    def visualize_depth(self, depth_map, filename):
        # Normalize the depth map for visualization
        depth_min = np.min(depth_map)
        depth_max = np.max(depth_map)
        depth_normalized = (depth_map - depth_min) / (depth_max - depth_min)

        # Apply a color map to the normalized depth map
        depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)

        # Display the depth map
        cv2.imwrite(f"data/preprocessed_test/{filename}", depth_colored)

    def depth_to_point_cloud(self, depth_map, image, cx, cy, focal_length):
        h, w = depth_map.shape[:2]
        i, j = np.meshgrid(np.arange(w), np.arange(h), indexing='xy')
        z = depth_map
        x = (i - cx) * z / focal_length
        y = (j - cy) * z / focal_length
        point_cloud = np.stack((x, y, z), axis=-1)

        # Get the colors from the image
        colors = image[j, i, :] / 255.0  # Normalize to [0, 1]
        point_cloud_with_colors = np.concatenate((point_cloud, colors), axis=-1)
        return point_cloud_with_colors

    def save_point_cloud(self, point_cloud, filename):
        # Convert the numpy array to an Open3D point cloud
        points = point_cloud[:, :, :3].reshape(-1, 3)
        colors = point_cloud[:, :, 3:].reshape(-1, 3)

        # Convert the numpy array to an Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Save the point cloud to a file
        o3d.io.write_point_cloud(filename, pcd)

        # o3d.visualization.draw_plotly([pcd])

def main(args=None):
   rclpy.init(args=args)

   madpose_solver = MADPoseSolver()
  
   # executor = MultiThreadedExecutor()
   # executor.add_node(moge_inference)


   # try:
   #     executor.spin()
   # finally:
   #     moge_inference.destroy_node()
   #     rclpy.shutdown()


   rclpy.spin(madpose_solver)


   # Destroy the node explicitly
   # (optional - otherwise it will be done automatically
   # when the garbage collector destroys the node object)
   madpose_solver.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()