#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import math

class VisualDocker(Node):
    def __init__(self):
        super().__init__('visual_docker')
        
        # Declare parameters to support use_sim_time (if not already declared via CLI)
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
        # State definitions
        self.IDLE = 0
        self.SERVOING = 1
        self.EXITING = 2
        self.DONE = 3
        self._state = self.IDLE
        
        # Watchdog config
        self.TAG_TIMEOUT = 1.5  # seconds
        self._last_pose_time = self.get_clock().now()
        self._has_received_pose = False
        
        # Controller configuration
        self.Kp_z = 0.6
        self.Kp_x = 0.8
        self.Kp_yaw = 0.5
        self.max_v = 0.08
        self.max_w = 0.15
        self.target_z = 0.15
        self.target_x = 0.0
        
        self._current_pose = None

        # ROS Interfaces
        self.create_subscription(PoseStamped, '/docking/tag_pose', self._tag_pose_callback, 10)
        self.create_subscription(Bool, '/docking/activate', self._activate_callback, 10)
        
        # amr_simulation/launch/sim.launch.py's WebotsController driver
        # already remaps diffdrive_controller's real input topic to plain
        # /cmd_vel.
        self._twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Unified Control and Watchdog Loop (20 Hz)
        self._control_timer = self.create_timer(0.05, self._control_loop)
        self._exit_start_time = None
        
        self.get_logger().info("visual_docker ready — publish True to /docking/activate to start")

    def _activate_callback(self, msg):
        if msg.data:
            if self._state == self.IDLE:
                self._state = self.SERVOING
                # Force reset to the EXACT current simulation time
                self._last_pose_time = self.get_clock().now()
                self._has_received_pose = False
                now_sec = self._last_pose_time.nanoseconds / 1e9
                self.get_logger().info(f"Docking ACTIVATED — SERVOING (Clock Reset to: {now_sec:.3f}s)")
        else:
            if self._state != self.IDLE:
                self._state = self.IDLE
                self._stop_robot()
                self.get_logger().info("Docking DEACTIVATED — entering IDLE state")

    def _tag_pose_callback(self, msg):
        # Update watchdog using the current node time when the message is RECEIVED
        self._last_pose_time = self.get_clock().now()
        self._has_received_pose = True
        self._current_pose = msg

    def _control_loop(self):
        if self._state == self.SERVOING:
            now = self.get_clock().now()
            
            # Calculate elapsed time strictly using the ROS Clock subtraction
            elapsed = (now - self._last_pose_time).nanoseconds / 1e9
            
            # Watchdog check
            if elapsed > self.TAG_TIMEOUT:
                now_sec = now.nanoseconds / 1e9
                last_sec = self._last_pose_time.nanoseconds / 1e9
                self.get_logger().warn(
                    f"Tag lost (Elapsed: {elapsed:.3f}s | Current: {now_sec:.3f}s | Last Seen: {last_sec:.3f}s) — reverting to IDLE"
                )
                self._state = self.IDLE
                self._stop_robot()
                return

            if not self._has_received_pose:
                # Keep robot still until the very first camera frame registers a tag
                self._stop_robot()
                return

            # Execute Proportional control calculations
            pose = self._current_pose.pose
            curr_x = pose.position.x
            curr_z = pose.position.z
            
            # Extract relative yaw
            qz = pose.orientation.z
            qw = pose.orientation.w
            curr_yaw = 2.0 * math.atan2(qz, qw)
            
            # Errors
            err_z = curr_z - self.target_z
            err_x = curr_x - self.target_x
            err_yaw = 0.0 - curr_yaw
            
            # Control speeds
            vx = self.Kp_z * err_z
            vy = self.Kp_x * err_x
            wz = self.Kp_yaw * err_yaw
            
            # Saturate velocities
            vx = max(min(vx, self.max_v), -self.max_v)
            vy = max(min(vy, self.max_v), -self.max_v)
            wz = max(min(wz, self.max_w), -self.max_w)
            
            self._publish_twist(vx, vy, wz)
            
            # Check tolerances to transition to Exit Phase (1.5 cm precision, 3 degrees)
            if abs(err_z) < 0.015 and abs(err_x) < 0.015 and abs(err_yaw) < 0.05:
                self.get_logger().info("DOCKING COMPLETE — Entering Exit State")
                self._state = self.EXITING
                self._exit_start_time = self.get_clock().now()

        elif self._state == self.EXITING:
            now = self.get_clock().now()
            exit_elapsed = (now - self._exit_start_time).nanoseconds / 1e9
            
            if exit_elapsed < 4.5:
                # Glide straight forward out of the corridor
                self._publish_twist(0.10, 0.0, 0.0)
            else:
                self.get_logger().info("EXIT COMPLETE — Docking node Idle")
                self._state = self.DONE
                self._stop_robot()

        elif self._state == self.DONE:
            self._stop_robot()

    def _publish_twist(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._twist_pub.publish(msg)
        self._cmd_vel_pub.publish(msg)

    def _stop_robot(self):
        self._publish_twist(0.0, 0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = VisualDocker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
