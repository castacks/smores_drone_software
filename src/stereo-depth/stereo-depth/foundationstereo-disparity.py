import numpy as np
import rclpy
import std_msgs.msg
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from cv_bridge import CvBridge
# import pcl2
import os, sys

from std_msgs.msg import Header

# code_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(
    "/home/aayush/prj/smores_drone_software/src/stereo-depth/stereo-depth/FoundationStereo"
)
from omegaconf import OmegaConf
from core.utils.utils import InputPadder
from Utils import *
from core.foundation_stereo import *
from parameters import *
import cv2

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

code_dir = os.path.dirname(os.path.realpath(__file__))
parser = argparse.ArgumentParser()
# parser.add_argument('--left_file', default=f'{code_dir}/../assets/left.png', type=str)
# parser.add_argument('--right_file', default=f'{code_dir}/../assets/right.png', type=str)
# parser.add_argument('--intrinsic_file', default=f'{code_dir}/../assets/K.txt', type=str,
#                     help='camera intrinsic matrix and baseline file')
parser.add_argument(
    "--ckpt_dir",
    default="/home/aayush/prj/smores_drone_software/src/stereo-depth/stereo-depth/FoundationStereo/pretrained_models/23-51-11/model_best_bp2.pth",
    type=str,
    help="pretrained model path",
)
# parser.add_argument('--out_dir', default=f'{code_dir}/../output/', type=str, help='the directory to save results')
parser.add_argument(
    "--scale", default=1, type=float, help="downsize the image by scale, must be <=1"
)
parser.add_argument(
    "--hiera",
    default=0,
    type=int,
    help="hierarchical inference (only needed for high-resolution images (>1K))",
)
parser.add_argument(
    "--z_far", default=10, type=float, help="max depth to clip in point cloud"
)
parser.add_argument(
    "--valid_iters",
    type=int,
    default=32,
    help="number of flow-field updates during forward pass",
)
parser.add_argument("--get_pc", type=int, default=1, help="save point cloud output")
parser.add_argument(
    "--remove_invisible",
    default=1,
    type=int,
    help="remove non-overlapping observations between left and right images from point cloud, so the remaining points are more reliable",
)
parser.add_argument(
    "--denoise_cloud", type=int, default=1, help="whether to denoise the point cloud"
)
parser.add_argument(
    "--denoise_nb_points",
    type=int,
    default=30,
    help="number of points to consider for radius outlier removal",
)
parser.add_argument(
    "--denoise_radius",
    type=float,
    default=0.03,
    help="radius to use for outlier removal",
)
args = parser.parse_args()

set_seed(0)
torch.autograd.set_grad_enabled(False)

ckpt_dir = "/home/aayush/prj/smores_drone_software/src/stereo-depth/stereo-depth/FoundationStereo/pretrained_models/23-51-11/model_best_bp2.pth"
cfg = OmegaConf.load(
    "/home/aayush/prj/smores_drone_software/src/stereo-depth/stereo-depth/FoundationStereo/pretrained_models/23-51-11/cfg.yaml"
)
for k in args.__dict__:
    cfg[k] = args.__dict__[k]
args = OmegaConf.create(cfg)

model = FoundationStereo(args)

ckpt = torch.load(ckpt_dir)
model.load_state_dict(ckpt["model"])
model.cuda()
model.eval()

padder = InputPadder((512, 640), divis_by=32, force_square=False)

# vis = o3d.visualization.Visualizer()
# vis.create_window()
  # Setting the look-at point to the origin

# def convert_open3d_to_pointcloud2(self, open3d_cloud, frame_id="map"):
#     points = np.asarray(open3d_cloud.points)
#     header = std_msgs.msg.Header()
#     header.stamp = self.get_clock().now().to_msg()
#     header.frame_id = frame_id
#     pointcloud2_msg = create_cloud_xyz32(header, points)
#     return pointcloud2_msg

def getdepth(left_img, right_img):
    model.eval()
    left_img_orig = np.stack([left_img] * 3, axis=-1)
    left_img = np.asarray([left_img] * 3).astype(np.float32)
    right_img = np.asarray([right_img] * 3).astype(np.float32)

    # disp_ests = model(torch.tensor(left_img).cuda(), torch.tensor(right_img).cuda())
    # disparity = disp_ests[0][0].cpu().detach().numpy()
    print(left_img.shape)
    img0 = torch.as_tensor(left_img).cuda().float()[None]  # .permute(0, 3, 1, 2)
    img1 = torch.as_tensor(right_img).cuda().float()[None]  # .permute(0, 3, 1, 2)
    img0, img1 = padder.pad(img0, img1)
    with torch.cuda.amp.autocast(True):
        disp = model.forward(img0, img1, iters=args.valid_iters, test_mode=True)
    disp = padder.unpad(disp.float())
    disp = disp.data.cpu().numpy().reshape(512, 640)
    disparity_norm = 255 * disp / 150
    disparity_norm = np.clip(disparity_norm, 0, 255)
    disparity_normalized_u8 = disparity_norm.astype(np.uint8)
    disparity_normalized_u8[disp < 0] = 0
    # K[:2] *= scale
    depth = K1[0, 0] * 0.24262 / disp
    # np.save(f"{args.out_dir}/depth_meter.npy", depth)
    xyz_map = depth2xyzmap(depth, K1)
    pcd = toOpen3dCloud(xyz_map.reshape(-1, 3), left_img_orig.reshape(-1, 3))
    keep_mask = (np.asarray(pcd.points)[:, 2] > 0) & (
            np.asarray(pcd.points)[:, 2] <= args.z_far
    )
    keep_ids = np.arange(len(np.asarray(pcd.points)))[keep_mask]
    pcd = pcd.select_by_index(keep_ids)
    # o3d.io.write_point_cloud(f"{args.out_dir}/cloud.ply", pcd)
    # logging.info(f"PCL saved to {args.out_dir}")

    # if args.denoise_cloud:
    #     logging.info("denoise point cloud...")
    #     cl, ind = pcd.remove_radius_outlier(
    #         nb_points=args.denoise_nb_points, radius=args.denoise_radius
    #     )
    #     inlier_cloud = pcd.select_by_index(ind)
    #     # o3d.io.write_point_cloud(f"{args.out_dir}/cloud_denoise.ply", inlier_cloud)
    #     pcd = inlier_cloud

    # logging.info("Visualizing point cloud. Press ESC to exit.")
    # vis.clear_geometries()
    # vis.add_geometry(pcd)
    # vis.get_render_option().point_size = 0.3
    # view_ctl = vis.get_view_control()
    # view_ctl.set_front([0, 0, 1])  # Looking along the positive Z-axis
    # view_ctl.set_up([0, -1, 0])  # Setting the up direction to negative Y-axis
    # view_ctl.set_lookat([0, 0, 10])
    # view_ctl.set_zoom(0.0625)
    # vis.get_render_option().background_color = np.array([0, 0, 0])
    # vis.run()

    # disparity_normalized_u8 = cv2.normalize(
    #     disp,
    #     None,
    #     alpha=0,
    #     beta=255,
    #     norm_type=cv2.NORM_MINMAX,
    #     dtype=cv2.CV_8U,
    # )
    return disparity_normalized_u8, pcd


class SyncDepthPair(Node):
    def __init__(self):
        super().__init__("sync_depth_pair")

        left_sub = message_filters.Subscriber(self, Image, "/thermal_left/image")
        right_sub = message_filters.Subscriber(self, Image, "/thermal_right/image")

        ats = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=10, slop=0.002
        )
        ats.registerCallback(self.pub_synced_pair)
        self.disparity_publisher = self.create_publisher(
            Image, "/thermal/disparityv2", 10
        )
        self.left_publisher = self.create_publisher(Image, "/thermal_left/clean", 10)
        self.right_publisher = self.create_publisher(Image, "/thermal_right/clean", 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/thermal/pointcloud', 10)
        self.bridge = CvBridge()

    def convert_open3d_to_pointcloud2(self, open3d_cloud, frame_id="map"):
        points = np.asarray(open3d_cloud.points)
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        pointcloud2_msg = create_cloud_xyz32(header, points)
        return pointcloud2_msg

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
        disparity_normalized_u8, pcd = getdepth(img_left_u8, img_right_u8)
        pclmsg = self.convert_open3d_to_pointcloud2(pcd, "map")
        self.disparity_publisher.publish(
            self.bridge.cv2_to_imgmsg(disparity_normalized_u8)
        )
        self.left_publisher.publish(self.bridge.cv2_to_imgmsg(img_left_u8))
        self.right_publisher.publish(self.bridge.cv2_to_imgmsg(img_right_u8))

        # pointcloud2_msg = self.convert_open3d_to_pointcloud2(pcd)
        self.pointcloud_publisher.publish(pclmsg)
        cv2.imshow("left", img_left_u8)
        cv2.imshow("right", img_right_u8)
        cv2.imshow("disparity", disparity_normalized_u8)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    sync_pair = SyncDepthPair()
    rclpy.spin(sync_pair)
    sync_pair.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
