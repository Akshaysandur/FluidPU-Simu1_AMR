#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

class SensorWrapperNode(Node):
    def __init__(self):
        super().__init__('sensor_wrapper_node')
        
        # Odom Sub & Pub (Injects non-zero covariances to prevent EKF division-by-zero)
        self.odom_sub = self.create_subscription(Odometry, '/rf2o/odom', self.odom_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/rf2o/odom_clean', 10)
        
        # IMU Sub & Pub (Injects frame_id, covariances, and cleans NaNs)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_clean', 10)
        
        self.get_logger().info('Unified Sensor Wrapper Node started successfully!')

    def odom_callback(self, msg):
        clean_msg = msg
        
        # --- VELOCITY DEADBAND FILTER ---
        # Clamp tiny scan-matching jitter to exactly 0.0 when practically stationary
        linear_x = clean_msg.twist.twist.linear.x
        linear_y = clean_msg.twist.twist.linear.y
        angular_z = clean_msg.twist.twist.angular.z
        
        if abs(linear_x) < 0.01:
            clean_msg.twist.twist.linear.x = 0.0
        if abs(linear_y) < 0.01:
            clean_msg.twist.twist.linear.y = 0.0
        if abs(angular_z) < 0.015:
            clean_msg.twist.twist.angular.z = 0.0
        # --------------------------------
        # Inject healthy covariances
        clean_msg.pose.covariance = [
            0.02, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.02, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.02, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.02, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.02
        ]
        clean_msg.twist.covariance = [
            0.002, 0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.002, 0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   0.002, 0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   0.002, 0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   0.002, 0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.002
        ]
        self.odom_pub.publish(clean_msg)

    def imu_callback(self, msg):
        clean_msg = msg
        clean_msg.header.frame_id = "imu_link"
        
        # Inject non-zero covariances for the IMU to allow correct EKF fusion
        clean_msg.orientation_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01
        ]
        clean_msg.angular_velocity_covariance = [
            0.001, 0.0,   0.0,
            0.0,   0.001, 0.0,
            0.0,   0.0,   0.001
        ]
        
        # Replace Linear Acceleration NaNs with safe zero values
        if math.isnan(clean_msg.linear_acceleration.x):
            clean_msg.linear_acceleration.x = 0.0
        if math.isnan(clean_msg.linear_acceleration.y):
            clean_msg.linear_acceleration.y = 0.0
        if math.isnan(clean_msg.linear_acceleration.z):
            clean_msg.linear_acceleration.z = 0.0
            
        clean_msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]
            
        self.imu_pub.publish(clean_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorWrapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
