#!/usr/bin/env python3
"""Mirrors the Webots-simulated AMR onto the real Dynamixel MX-106R wheels.

Subscribes to the SAME /cmd_vel topic that drives the simulated robot's wheels in Webots (the final,
post-collision-monitor command - see amr_navigation/config/nav2_params.yaml's collision_monitor comment).
Nav2 keeps planning and driving purely against the simulated robot and map; this node's only job is to make
the physical wheels replay that command in real time. It does NOT localize or avoid obstacles on its own -
that is what the ultrasonic_node's /hw_estop is for (a real-world safety net, since the sim has no idea
what is actually in front of the physical robot).

Differential-drive kinematics: v_left = linear.x - angular.z * wheel_separation / 2
                                v_right = linear.x + angular.z * wheel_separation / 2
converted to wheel angular velocity (rad/s) by /wheel_radius, then to Dynamixel raw velocity units
(0.229 rpm/unit, Protocol 2.0) via the SDK's velocity conversion.
"""
import math
import threading

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from dynamixel_sdk import COMM_SUCCESS, GroupSyncWrite, PacketHandler, PortHandler
except ImportError:                          # allows `ros2 run` --help / linting off the Pi
    PortHandler = PacketHandler = GroupSyncWrite = None
    COMM_SUCCESS = 0

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
VELOCITY_UNIT_RPM = 0.229                     # Dynamixel Protocol 2.0 raw unit -> rpm, MX-106
TICKS_PER_REV = 4096


class DynamixelDrive(Node):
    def __init__(self):
        super().__init__('dynamixel_drive')
        d = self.declare_parameter
        d('device', '/dev/dynamixel_u2d2')
        d('baudrate', 1000000)
        d('protocol_version', 2.0)
        d('left_id', 1)
        d('right_id', 2)
        d('operating_mode', 1)
        d('wheel_radius', 0.033)
        d('wheel_separation', 0.16)
        d('max_wheel_rpm', 45.0)
        d('cmd_vel_timeout', 0.5)
        d('publish_odom', True)
        g = self.get_parameter
        self.left_id, self.right_id = g('left_id').value, g('right_id').value
        self.wheel_radius = g('wheel_radius').value
        self.wheel_sep = g('wheel_separation').value
        self.max_wheel_rad_s = g('max_wheel_rpm').value * 2 * math.pi / 60.0
        self.timeout = g('cmd_vel_timeout').value

        self.estopped = False
        self.lock = threading.Lock()
        self.last_cmd_time = self.get_clock().now()

        if PortHandler is None:
            self.get_logger().error(
                "dynamixel_sdk not installed (pip install dynamixel-sdk). Node is up but cannot drive motors.")
            self.port = self.packet = None
        else:
            self.port = PortHandler(g('device').value)
            self.packet = PacketHandler(g('protocol_version').value)
            if not self.port.openPort() or not self.port.setBaudRate(g('baudrate').value):
                self.get_logger().error(f"Could not open {g('device').value} @ {g('baudrate').value} bps")
                self.port = None
            else:
                for mid in (self.left_id, self.right_id):
                    self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, 0)
                    self.packet.write1ByteTxRx(self.port, mid, ADDR_OPERATING_MODE, g('operating_mode').value)
                    self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, 1)
                self.get_logger().info(f"Dynamixel bus up on {g('device').value}, ids {self.left_id}/{self.right_id}")

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Bool, '/hw_estop', self.on_estop, 10)
        if g('publish_odom').value:
            self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        else:
            self.odom_pub = None
        self.create_timer(0.05, self.watchdog_tick)        # 20 Hz: stop on stale /cmd_vel, poll encoders

    def on_estop(self, msg):
        self.estopped = msg.data
        if self.estopped:
            self.get_logger().warn('/hw_estop TRUE - obstacle inside the safety zone, stopping the real wheels '
                                   '(the simulated robot keeps going; this is a real-world-only safety stop)')
            self.write_wheel_velocity(0.0, 0.0)

    def on_cmd_vel(self, msg):
        self.last_cmd_time = self.get_clock().now()
        if self.estopped:
            return
        v_left = msg.linear.x - msg.angular.z * self.wheel_sep / 2.0
        v_right = msg.linear.x + msg.angular.z * self.wheel_sep / 2.0
        self.write_wheel_velocity(v_left, v_right)

    def write_wheel_velocity(self, v_left_ms, v_right_ms):
        """v_*_ms: linear surface speed of each wheel, m/s."""
        if self.port is None:
            return
        w_left = max(-self.max_wheel_rad_s, min(self.max_wheel_rad_s, v_left_ms / self.wheel_radius))
        w_right = max(-self.max_wheel_rad_s, min(self.max_wheel_rad_s, v_right_ms / self.wheel_radius))
        raw_left = int(round((w_left * 60.0 / (2 * math.pi)) / VELOCITY_UNIT_RPM))
        raw_right = -int(round((w_right * 60.0 / (2 * math.pi)) / VELOCITY_UNIT_RPM))   # right motor is inverted
        with self.lock:
            self.packet.write4ByteTxRx(self.port, self.left_id, ADDR_GOAL_VELOCITY, raw_left & 0xFFFFFFFF)
            self.packet.write4ByteTxRx(self.port, self.right_id, ADDR_GOAL_VELOCITY, raw_right & 0xFFFFFFFF)

    def watchdog_tick(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age > self.timeout:
            self.write_wheel_velocity(0.0, 0.0)          # link dropped or sim stopped publishing: stop, don't coast

    def destroy_node(self):
        self.write_wheel_velocity(0.0, 0.0)
        if self.port is not None:
            for mid in (self.left_id, self.right_id):
                self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, 0)
            self.port.closePort()
        super().destroy_node()


def main():
    rclpy.init()
    node = DynamixelDrive()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
