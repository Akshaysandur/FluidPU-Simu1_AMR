#!/usr/bin/env python3
"""HC-SR04 -> sensor_msgs/Range on /ultrasonic_front, and the real-world safety net for mirror mode.

The simulated robot's Nav2 stack has no idea what is actually in front of the PHYSICAL robot (it only
avoids what is in the simulated map). This node is the one thing standing between a mismatched sim/real
layout and the real robot driving into something: whenever the sonar reads inside `safety_stop_dist`, it
publishes True on /hw_estop, which dynamixel_drive.py treats as an immediate stop regardless of what
/cmd_vel says. It publishes False again the instant the obstacle clears - it does not resume motion itself,
the mirrored /cmd_vel does that on its own once the (matching) sim path is clear too.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

try:
    from gpiozero import DistanceSensor
except ImportError:
    DistanceSensor = None


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        d = self.declare_parameter
        d('trigger_pin', 23)
        d('echo_pin', 24)
        d('frame_id', 'ultrasonic_front_link')
        d('rate_hz', 15.0)
        d('safety_stop_dist', 0.15)
        d('safety_topic', '/hw_estop')
        self.frame_id = self.get_parameter('frame_id').value
        self.stop_dist = self.get_parameter('safety_stop_dist').value
        self.was_blocked = False

        self.range_pub = self.create_publisher(Range, '/ultrasonic_front', 10)
        self.estop_pub = self.create_publisher(Bool, self.get_parameter('safety_topic').value, 10)
        if DistanceSensor is None:
            self.get_logger().error('gpiozero not installed; ultrasonic safety net is NOT active')
            self.sensor = None
        else:
            self.sensor = DistanceSensor(
                echo=self.get_parameter('echo_pin').value, trigger=self.get_parameter('trigger_pin').value,
                max_distance=4.0)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def tick(self):
        if self.sensor is None:
            return
        dist = self.sensor.distance                       # metres
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASONIC
        msg.field_of_view = 0.26                            # ~15 degrees
        msg.min_range, msg.max_range = 0.02, 4.0
        msg.range = dist
        self.range_pub.publish(msg)

        blocked = dist < self.stop_dist
        if blocked != self.was_blocked:
            self.estop_pub.publish(Bool(data=blocked))
            self.get_logger().info(f'ultrasonic {dist:.2f} m: {"BLOCKED" if blocked else "clear"}')
            self.was_blocked = blocked


def main():
    rclpy.init()
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
