#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomCovarianceWrapper(Node):
    def __init__(self):
        super().__init__('odom_covariance_wrapper')
        self.sub = self.create_subscription(Odometry, '/rf2o/odom', self.callback, 10)
        self.pub = self.create_publisher(Odometry, '/rf2o/odom_with_covariance', 10)
        self.get_logger().info('Odom Covariance Wrapper Node successfully started!')

    def callback(self, msg):
        new_msg = msg
        # Inject healthy, non-zero covariance diagonals to prevent EKF division-by-zero
        new_msg.pose.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.01
        ]
        new_msg.twist.covariance = [
            0.001, 0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.001, 0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   0.001, 0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   0.001, 0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   0.001, 0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.001
        ]
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomCovarianceWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
