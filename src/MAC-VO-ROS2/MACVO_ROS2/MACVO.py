import rclpy
import torch
import pypose as pp

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time

from pathlib import Path
from typing import TYPE_CHECKING
import os, sys
import argparse
import logging

import cv2

from .MessageFactory import to_stamped_pose, from_image, to_pointcloud, to_image

# Add the src directory to the Python path
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'src'))
sys.path.insert(0, src_path)
if TYPE_CHECKING:
    # To make static type checker happy : )
    from src.Odometry.MACVO import MACVO
    from src.DataLoader import StereoFrame, StereoData, SmartResizeFrame
    from src.Utility.Config import load_config
    from src.Utility.PrettyPrint import Logger
    from src.Utility.Timer import Timer
else:
    import DataLoader
    from Odometry.MACVO import MACVO                
    from DataLoader import StereoFrame, StereoData, SmartResizeFrame
    from Utility.Config import load_config
    from Utility.PrettyPrint import Logger
    from Utility.Timer import Timer

PACKAGE_NAME = "MACVO_ROS2"

# MAC-VO Node


class MACVONode(Node):
    def __init__(
        self, config: Path,
        imageL_subscribe: str,  # Rectified RGB image, expect to receive sensor_msg/image
        imageR_subscribe: str,  # Rectified RGB image, expect to receive sensor_msg/image

        pose_publish: str,      # Estimated pose of Left camera optical center, NED coordinate, geometry_msg/pose_stamped
        map_pc_publish: str,    # Dense mapping pointcloud, sensor_msg/pointcloud 
        imageL_publish: str     # Scaled & cropped RGB image actually received by MAC-VO, sensor_msg/image
    ) -> None:
        super().__init__("macvo")
        self.get_logger().set_level(logging.INFO)
        self.get_logger().info(f"{os.getcwd()}")

        imageL_recv = Subscriber(self, Image, imageL_subscribe, qos_profile=1)
        imageR_recv = Subscriber(self, Image, imageR_subscribe, qos_profile=1)
        self.stereo_recv = ApproximateTimeSynchronizer([imageL_recv, imageR_recv], queue_size=2, slop=0.1)
        self.stereo_recv.registerCallback(self.receive_stereo)

        self.pose_send = self.create_publisher(PoseStamped, pose_publish, qos_profile=1)
        self.map_send  = self.create_publisher(PointCloud, map_pc_publish, qos_profile=1)
        self.img_send  = self.create_publisher(Image, imageL_publish, qos_profile=1)
        
        # Start MAC-VO
        cfg, _ = load_config(config)
        self.frame_id, self.camera = 0, cfg.Camera
        
        # Load model from the resources directory automatically.
        original_cwd = os.getcwd()
        try:
            os.chdir(get_package_share_directory(PACKAGE_NAME))
            self.get_logger().info(get_package_share_directory(PACKAGE_NAME))
            self.odometry = MACVO[StereoFrame].from_config(cfg)
        finally:
            os.chdir(original_cwd)
        # End
        self.frame_fn = SmartResizeFrame({"height": 320, "width": 320, "interp": "bilinear"})
        #

        self.odometry.register_on_optimize_finish(self.publish_data)
        
        self.time, self.prev_time = None, None
        self.coord_frame = "thermal_left/optical_frame"  #FIXME: this is probably wrong?
        

    def publish_data(self, system: MACVO):
        # Latest pose
        pose    = pp.SE3(system.graph.frames.data["pose"][-1])
        time_ns = int(system.graph.frames.data["time_ns"][-1].item())
        time = Time()
        time.sec = time_ns // 1_000_000_000
        time.nanosec = time_ns % 1_000_000_000
        pose_msg = to_stamped_pose(pose, self.coord_frame, time)
        
        # Latest map
        if system.mapping:
            points = system.graph.get_frame2map(system.graph.frames[-2:-1])
        else:
            points = system.graph.get_match2point(system.graph.get_frame2match(system.graph.frames[-1:]))

        map_pc_msg = to_pointcloud(
            position  = points.data["pos_Tw"],
            keypoints = None,
            frame_id  = self.coord_frame,
            colors    = points.data["color"],
            time      = time,
        )
        
        self.pose_send.publish(pose_msg)
        self.map_send.publish(map_pc_msg)

    def receive_stereo(self, msg_imageL: Image, msg_imageR: Image) -> None:
        self.get_logger().info(f"{self.odometry.graph}")
        imageL, timestamp = from_image(msg_imageL), msg_imageL.header.stamp
        imageR            = from_image(msg_imageR)

        imageL = cv2.cvtColor(imageL, cv2.COLOR_BGR2RGB)
        imageR = cv2.cvtColor(imageR, cv2.COLOR_BGR2RGB)
        
        # Instantiate a frame and scale to the desired height & width
        stereo_frame = self.frame_fn(StereoFrame(
            idx    =[self.frame_id],
            time_ns=[timestamp.nanosec],
            stereo =StereoData(
                T_BS=pp.identity_SE3(1),
                K   =torch.tensor([[
                    [self.camera.fx, 0.            , self.camera.cx],
                    [0.            , self.camera.fy, self.camera.cy],
                    [0.            , 0.            , 1.            ]
                ]]),
                baseline=torch.tensor([self.camera.bl]),
                time_ns=[timestamp.nanosec],
                height=imageL.shape[0],
                width=imageL.shape[1],
                imageL=torch.tensor(imageL)[..., :3].float().permute(2, 0, 1).unsqueeze(0) / 255.,
                imageR=torch.tensor(imageR)[..., :3].float().permute(2, 0, 1).unsqueeze(0) / 255.,
            )
        ))
        self.odometry.run(stereo_frame)
        
        # Pose-processing
        self.frame_id += 1
    
    def destroy_node(self):
        self.odometry.terminate()


def main():
    rclpy.init()
    args = argparse.ArgumentParser()
    args.add_argument("--config", type=str, default="./config/flirboson_config.yaml")
    args.add_argument("--timing", action="store_true", help="Record timing for system (active Utility.Timer for global time recording)")
    args = args.parse_args()
    
    # Optional - setup Timer to see profiling result
    Timer.setup(active=args.timing)
    # Optional End
    
    # Create Node and start running
    node = MACVONode(
        config=Path(args.config),
        #imageL_subscribe="/zed/zed_node/left/image_rect_color",
        #imageR_subscribe="/zed/zed_node/right/image_rect_color",
        imageL_subscribe="/thermal_left/preprocd_image",
        imageR_subscribe="/thermal_right/preprocd_image",
        pose_publish    ="/macvo/pose",
        map_pc_publish  ="/macvo/map",
        imageL_publish  ="/macvo/img",
    )
    print('MACVO Node created.')
    rclpy.spin(node)
    # End
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()
