#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
import numpy as np
from numpy import typing as npt
import tf_transformations as tft
from typing import List, Tuple


def normalize_quaternion(q: List[float]) -> List[float]:
    arr = np.asarray(q, dtype=float)
    n = np.linalg.norm(arr)
    if n == 0.0:
        raise ValueError("Zero-length quaternion provided")
    return (arr / n).tolist()


def quaternion_to_rotation_matrix(q: List[float]) -> npt.NDArray[np.float64]:
    """Return 3x3 rotation matrix from quaternion [x,y,z,w]."""
    qm = tft.quaternion_matrix(q)  # 4x4 homogeneous
    return np.asarray(qm[:3, :3], dtype=np.float64)


def rotate_covariance_6x6(
    cov6: npt.NDArray[np.float64], rotation_3x3: npt.NDArray[np.float64]
) -> npt.NDArray[np.float64]:
    """Rotate a 6x6 covariance matrix where ordering is [x,y,z,rot_x,rot_y,rot_z].
    Uses block-diagonal transform A = diag(R, R): cov' = A cov A^T.

    This is an approximation for orientation blocks (treats orientation like a 3-vector).
    For many use-cases this is acceptable; strictly speaking orientation covariance
    needs more careful handling (e.g. using minimal-angle parametrization).
    """
    if cov6.shape != (6, 6):
        raise ValueError("cov6 must be shape (6,6)")
    A = np.zeros((6, 6), dtype=np.float64)
    A[:3, :3] = rotation_3x3
    A[3:, 3:] = rotation_3x3
    return A @ cov6 @ A.T


class OdomTransformer(Node):
    """ROS2 node that applies a fixed coordinate transform to incoming odometry and republishes.

    The fixed transform maps the incoming odometry pose expressed in the *sensor*
    frame into the *odom* frame using:
        p_odom = R * p_sensor + t
        q_odom = q_rot * q_sensor

    Where:
        - rotation_quaternion = q_rot (sensor -> odom)
        - translation_vector = t (expressed in odom frame after applying R)
    """

    def __init__(self) -> None:
        super().__init__("odom_transformer")

        # Translation (meters) and rotation quaternion [x,y,z,w] mapping sensor -> odom.
        self.translation_vector: npt.NDArray[np.float64] = np.array([0.0, 0.0, 0.0], dtype=float)
        raw_q: List[float] = [-0.5, 0.5, 0.5, 0.5]
        self.rotation_quaternion: List[float] = normalize_quaternion(raw_q)
        self.rotation_matrix: npt.NDArray[np.float64] = quaternion_to_rotation_matrix(self.rotation_quaternion)

        self.odom_subscriber = self.create_subscription(
            Odometry, "/rovtio/out", self.odom_callback, 10
        )
        self.odom_publisher = self.create_publisher(Odometry, "/mavros/odometry/out", 10)

        self.get_logger().info("OdomTransformer node started")

    def odom_callback(self, odom_in: Odometry) -> None:
        """Transform incoming Odometry (sensor frame) into odom frame and publish.

        Incoming assumptions:
            - odom_in.pose.pose.position: position of sensor origin in sensor frame
            - odom_in.pose.pose.orientation: orientation of the sensor in sensor frame
            - odom_in.twist.twist.linear: linear velocity expressed in sensor frame
            - odom_in.twist.twist.angular: angular velocity expressed in sensor frame

        Output:
            - odom_out with header.frame_id = "odom" and child_frame_id = "base_link"
              All vectors expressed in the odom frame.
        """
        p_in = np.array(
            [odom_in.pose.pose.position.x, 
             odom_in.pose.pose.position.y, 
             odom_in.pose.pose.position.z], dtype=float)

        q_in = [
            odom_in.pose.pose.orientation.x,
            odom_in.pose.pose.orientation.y,
            odom_in.pose.pose.orientation.z,
            odom_in.pose.pose.orientation.w,
        ]
        
        q_in = normalize_quaternion(q_in)

        v_linear_in = np.array(
            [odom_in.twist.twist.linear.x, odom_in.twist.twist.linear.y, odom_in.twist.twist.linear.z], dtype=float
        )

        omega_in = np.array(
            [odom_in.twist.twist.angular.x, odom_in.twist.twist.angular.y, odom_in.twist.twist.angular.z], dtype=float
        )

        p_out, q_out, v_linear_out, omega_out = self.apply_transform(
            p_in, q_in, v_linear_in, omega_in, self.rotation_quaternion, self.translation_vector
        )

        odom_out = Odometry()
        odom_out.header.stamp = odom_in.header.stamp
        odom_out.header.frame_id = "odom"
        odom_out.child_frame_id = "base_link"

        odom_out.pose.pose.position = Point(x=float(p_out[0]), y=float(p_out[1]), z=float(p_out[2]))
        odom_out.pose.pose.orientation = Quaternion(x=q_out[0], y=q_out[1], z=q_out[2], w=q_out[3])

        odom_out.twist.twist.linear.x = float(v_linear_out[0])
        odom_out.twist.twist.linear.y = float(v_linear_out[1])
        odom_out.twist.twist.linear.z = float(v_linear_out[2])
        odom_out.twist.twist.angular.x = float(omega_out[0])
        odom_out.twist.twist.angular.y = float(omega_out[1])
        odom_out.twist.twist.angular.z = float(omega_out[2])

        # Rotate and publish covariances if present (practical/approximate)
        try:
            pose_cov_in = np.asarray(odom_in.pose.covariance, dtype=float).reshape((6, 6))
            pose_cov_out = rotate_covariance_6x6(pose_cov_in, self.rotation_matrix)
            odom_out.pose.covariance = tuple(map(float, pose_cov_out.reshape(-1)))
        except Exception:
            self.get_logger().warn(f"Static tf pub node the error while handling pose covariance rotation")
            pass

        try:
            twist_cov_in = np.asarray(odom_in.twist.covariance, dtype=float).reshape((6, 6))
            twist_cov_out = rotate_covariance_6x6(twist_cov_in, self.rotation_matrix)
            odom_out.twist.covariance = tuple(map(float, twist_cov_out.reshape(-1)))
        except Exception:
            self.get_logger().warn(f"Static tf pub node the error while handling twist covariance rotation")

        self.odom_publisher.publish(odom_out)

    def apply_transform(
        self,
        position_in: npt.NDArray[np.float64],
        orientation_in: List[float],
        linear_velocity_in: npt.NDArray[np.float64],
        angular_velocity_in: npt.NDArray[np.float64],
        rotation_quaternion: List[float],
        translation_vector: npt.NDArray[np.float64],
    ) -> Tuple[npt.NDArray[np.float64], List[float], npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """Apply rotation + translation to pose and twist.

        Returns (position_out, orientation_out, linear_velocity_out, angular_velocity_out)
        All returned vectors are expressed in the odom frame.
        """
        # ensure normalized quaternion
        rotation_quaternion = normalize_quaternion(rotation_quaternion)
        orientation_in = normalize_quaternion(orientation_in)

        # rotate position and add translation
        R = quaternion_to_rotation_matrix(rotation_quaternion)
        position_out = R @ position_in + translation_vector

        # compose orientation: q_out = q_rot * q_in
        orientation_out = tft.quaternion_multiply(rotation_quaternion, orientation_in)
        orientation_out = normalize_quaternion(orientation_out)

        # rotate angular velocity
        angular_velocity_out = self.rotate_vector(angular_velocity_in, rotation_quaternion)

        # rotate linear velocity and add correction for translation if body is rotating:
        # v_out = R * v_in + omega_out x translation_vector
        linear_velocity_rotated = self.rotate_vector(linear_velocity_in, rotation_quaternion)
        linear_velocity_out = linear_velocity_rotated + np.cross(angular_velocity_out, translation_vector)

        return position_out, orientation_out, linear_velocity_out, angular_velocity_out

    def rotate_vector(self, vector: npt.NDArray[np.float64], quaternion: List[float]) -> npt.NDArray[np.float64]:
        """Rotate a 3D vector using quaternion (vector expressed in input frame -> output frame)."""
        q_conj = tft.quaternion_conjugate(quaternion)
        v_q = [float(vector[0]), float(vector[1]), float(vector[2]), 0.0]
        v_rot = tft.quaternion_multiply(tft.quaternion_multiply(quaternion, v_q), q_conj)
        return np.array(v_rot[:3], dtype=float)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
