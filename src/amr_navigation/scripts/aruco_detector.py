#!/usr/bin/env python3
"""
aruco_detector.py — Real-time ArUco pose estimator for visual docking.

Subscribes to '/amr_robot/camera/image_color', detects markers from
DICT_7X7_50 (IDs 11, 12, 21, 22, 31, 32), and publishes each
detected marker's 3D pose as a PoseStamped on '/docking/tag_pose'.

Intrinsics are hardcoded from the verified /amr_robot/camera/camera_info:
  Width: 424, Height: 240
  fx = fy = 223.40954070056824
  cx = 212.0, cy = 120.0
  D  = [0, 0, 0, 0, 0]
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


# ---------------------------------------------------------------------------
# Hardcoded intrinsics (verified from /amr_robot/camera/camera_info)
# ---------------------------------------------------------------------------
CAMERA_MATRIX = np.array(
    [
        [223.40954070056824, 0.0, 212.0],
        [0.0, 223.40954070056824, 120.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
DIST_COEFFS = np.zeros((5,), dtype=np.float64)

# Physical side-length of the ArUco marker in metres (100 mm)
MARKER_LENGTH = 0.10

# Only report poses for these docking-station marker IDs
TARGET_IDS = {11, 12, 21, 22, 31, 32}


def rvec_to_yaw(rvec: np.ndarray) -> float:
    """Convert a Rodrigues rotation vector to the yaw component (rad)."""
    rot_mat, _ = cv2.Rodrigues(rvec)
    # Extract yaw from R (rotation around Z in camera frame)
    yaw = math.atan2(rot_mat[1, 0], rot_mat[0, 0])
    return yaw


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # --- ArUco setup (OpenCV 4.5.x old API) -------------------------
        # Use DICT_7X7_50 — matches the physically generated markers
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
        self._aruco_params = cv2.aruco.DetectorParameters_create()

        # --- cv_bridge ---------------------------------------------------
        self._bridge = CvBridge()

        # --- ROS interfaces ----------------------------------------------
        self._image_sub = self.create_subscription(
            Image,
            '/amr_robot/camera/image_color',
            self._image_callback,
            10,
        )
        self._pose_pub = self.create_publisher(PoseStamped, '/docking/tag_pose', 10)

        self.get_logger().info('aruco_detector ready (DICT_7X7_50, targets=%s)' % TARGET_IDS)

    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge error: {exc}')
            return

        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self._aruco_dict, parameters=self._aruco_params
        )

        if ids is None:
            return

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id not in TARGET_IDS:
                continue

            # Estimate 3-D pose of this single marker
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[i]], MARKER_LENGTH, CAMERA_MATRIX, DIST_COEFFS
            )
            rvec = rvec[0][0]  # shape (3,)
            tvec = tvec[0][0]  # shape (3,)  [x_lateral, y_vertical, z_depth]

            yaw = rvec_to_yaw(rvec)

            # Build PoseStamped:
            #   position.x = lateral offset  (left/right)
            #   position.z = forward depth   (distance to marker)
            #   orientation.z = sin(yaw/2)   (quaternion shorthand)
            #   orientation.w = cos(yaw/2)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = f'tag_{marker_id}'
            pose_msg.pose.position.x = float(tvec[0])   # lateral
            pose_msg.pose.position.y = float(tvec[1])   # vertical (unused by docker)
            pose_msg.pose.position.z = float(tvec[2])   # forward depth
            pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
            pose_msg.pose.orientation.w = math.cos(yaw / 2.0)

            self._pose_pub.publish(pose_msg)

            self.get_logger().debug(
                f'[tag_{marker_id}] x={tvec[0]:.3f} z={tvec[2]:.3f} yaw={math.degrees(yaw):.1f}°'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
