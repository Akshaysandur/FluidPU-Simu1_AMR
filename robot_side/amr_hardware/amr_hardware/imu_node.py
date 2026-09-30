#!/usr/bin/env python3
"""BNO085 (Adafruit/CEVA 9-DOF IMU) -> sensor_msgs/Imu on /imu, fused quaternion + angular velocity.
Not consumed by Nav2 in mirror mode (the sim robot's own IMU drives navigation) - this is real telemetry,
useful once the hardware runs its own stack later, and for sanity-checking the real robot's heading now.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import board
    import busio
    from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_ROTATION_VECTOR
    from adafruit_bno08x.i2c import BNO08X_I2C
except ImportError:
    board = busio = BNO08X_I2C = None


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        d = self.declare_parameter
        d('i2c_bus', 1)
        d('i2c_address', 0x4A)
        d('frame_id', 'imu_link')
        d('rate_hz', 100.0)
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(Imu, '/imu', 10)
        if BNO08X_I2C is None:
            self.get_logger().error('adafruit-circuitpython-bno08x not installed; /imu will not publish')
            self.sensor = None
        else:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
            self.sensor = BNO08X_I2C(i2c, address=self.get_parameter('i2c_address').value)
            self.sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.sensor.enable_feature(BNO_REPORT_GYROSCOPE)
            self.sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def tick(self):
        if self.sensor is None:
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        qi, qj, qk, qr = self.sensor.quaternion
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = qi, qj, qk, qr
        gx, gy, gz = self.sensor.gyro
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx, gy, gz
        ax, ay, az = self.sensor.acceleration
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ax, ay, az
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
