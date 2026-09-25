#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def classify(vx: float, w: float) -> str:
    if vx < -0.05:
        return 'RECOVER_BACKUP'
    if abs(vx) <= 0.01 and abs(w) <= 0.01:
        return 'IDLE_STATIONARY'
    if vx > 0.05 and abs(w) <= 0.08:
        return 'STRAIGHT_FORWARD'
    if vx > 0.05 and w > 0.08:
        return 'CURVING_LEFT'
    if vx > 0.05 and w < -0.08:
        return 'CURVING_RIGHT'
    if abs(vx) <= 0.03 and abs(w) > 0.08:
        return 'IN_PLACE_SPIN'
    # Falls between the named bands (e.g. vx in (0.03, 0.05]) - not one of
    # the six requested states, so surfaced explicitly instead of silently
    # matching the wrong bucket.
    return 'TRANSITIONING'


class ManeuverTelemetryNode(Node):
    # Decision D (narrow-passage-navigation-fix-brief.md): a recovery-loop
    # storm doesn't just fail to progress, it also thrashes classify()
    # between states almost every /cmd_vel_raw message (IN_PLACE_SPIN <->
    # TRANSITIONING <-> IDLE_STATIONARY, observed live at close to
    # controller_frequency itself), and this node already logs on every
    # state CHANGE - so during exactly the failure this brief exists to
    # fix, that becomes a near-every-cycle flood, adding real I/O load on
    # top of a system that's already struggling. A minimum interval between
    # log lines caps that without losing the substantive signal (the
    # current state is still reported, just not faster than this).
    MIN_LOG_INTERVAL_S = 0.3

    def __init__(self):
        super().__init__('maneuver_telemetry_node')
        self.last_maneuver = None
        self.last_log_time = None
        self.create_subscription(Twist, '/cmd_vel_raw', self._cmd_vel_callback, 10)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        vx = msg.linear.x
        vy = msg.linear.y
        w = msg.angular.z

        state = classify(vx, w)
        now = self.get_clock().now()
        due = (
            self.last_log_time is None
            or (now - self.last_log_time).nanoseconds / 1e9 >= self.MIN_LOG_INTERVAL_S
        )
        if state != self.last_maneuver and due:
            self.last_maneuver = state
            self.last_log_time = now
            self.get_logger().info(
                f"[MANEUVER TELEMETRY] Active State: {state} | vx={vx:.2f} m/s, w={w:.2f} rad/s"
            )
        # vy is extracted per spec but unused in classification - on this
        # topic specifically it's always ~0.0 anyway, since FollowPath's
        # min_vel_y/max_vel_y are locked to 0.0 (see nav2_params.yaml).
        del vy


def main(args=None):
    rclpy.init(args=args)
    node = ManeuverTelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
