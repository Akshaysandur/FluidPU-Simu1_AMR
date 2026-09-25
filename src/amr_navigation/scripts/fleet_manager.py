#!/usr/bin/env python3
"""Minimal FMS: station command -> ordered Nav2 waypoint run.

Reads the LIF-style GeoJSON graph (Point features with properties.id, coordinates
in the map frame), and on a station command drives every waypoint of that station
in order, always starting from the first one regardless of where the robot is.
Each waypoint is a Nav2 NavigateToPose goal, so wall avoidance / recovery is
Nav2's (costmaps + collision_monitor + behavior_server). A waypoint is only
considered done when Nav2 succeeds AND the measured map->base_link pose is
within `arrive_tolerance`; otherwise it is re-sent (up to `max_retries`).

Triggers:  service  /fms/station_1 (std_srvs/Trigger)   [one per station]
           topic    /fms/command   (std_msgs/String, e.g. "station_1")
Status:    topic    /fms/status    (std_msgs/String)
"""
import json
import math
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        self.declare_parameter('graph_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('arrive_tolerance', 0.15)
        self.declare_parameter('arrive_yaw_tolerance', 0.6)
        self.declare_parameter('approach_dist', 0.30)    # staging point in front of the entry, straight line into the mouth
        self.declare_parameter('exit_clear_dist', 0.35)  # staging point beyond the exit so the robot really leaves the station
        self.declare_parameter('staging_tolerance', 0.15)
        self.declare_parameter('max_retries', 5)
        # LIF graph frame -> Nav2 map frame: map = offset + scale * lif.
        # Fitted from outer-wall positions of the LIF floor plan vs the SLAM map.
        self.declare_parameter('lif_scale_x', 1.0)
        self.declare_parameter('lif_scale_y', 1.0)
        self.declare_parameter('lif_offset_x', 0.0)
        self.declare_parameter('lif_offset_y', 0.0)
        self.declare_parameter('station_names', ['station_1'])
        self.declare_parameter('station_1_nodes', [18, 10, 17])

        with open(self.get_parameter('graph_file').value) as f:
            feats = json.load(f)['features']
        self.nodes = {
            ft['properties']['id']: (
                self.get_parameter('lif_offset_x').value
                + self.get_parameter('lif_scale_x').value * ft['geometry']['coordinates'][0],
                self.get_parameter('lif_offset_y').value
                + self.get_parameter('lif_scale_y').value * ft['geometry']['coordinates'][1])
            for ft in feats if ft['geometry']['type'] == 'Point'
        }
        self.stations = {
            n: list(self.get_parameter(f'{n}_nodes').value)
            for n in self.get_parameter('station_names').value
        }
        for n, ids in self.stations.items():
            missing = [i for i in ids if i not in self.nodes]
            if missing:
                raise RuntimeError(f'{n}: node ids {missing} not in graph')

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tol = self.get_parameter('arrive_tolerance').value
        self.yaw_tol = self.get_parameter('arrive_yaw_tolerance').value
        self.approach_dist = self.get_parameter('approach_dist').value
        self.exit_dist = self.get_parameter('exit_clear_dist').value
        self.staging_tol = self.get_parameter('staging_tolerance').value
        self.max_retries = self.get_parameter('max_retries').value

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self.status_pub = self.create_publisher(String, '/fms/status', 10)
        self.create_subscription(String, '/fms/command', self._on_command, 10, callback_group=cb)
        for n in self.stations:
            self.create_service(Trigger, f'/fms/{n}', lambda req, res, n=n: self._on_service(n, res),
                                callback_group=cb)
        self._busy = threading.Lock()
        self.get_logger().info(f'FMS ready. stations={self.stations}')

    # ---- triggers -------------------------------------------------------
    def _on_service(self, name, res):
        res.success, res.message = self._start(name)
        return res

    def _on_command(self, msg):
        ok, text = self._start(msg.data.strip())
        self.get_logger().info(text)

    def _start(self, name):
        if name not in self.stations:
            return False, f'unknown station "{name}"'
        if not self._busy.acquire(blocking=False):
            return False, 'FMS busy with another run'
        threading.Thread(target=self._run, args=(name,), daemon=True).start()
        return True, f'{name} started'

    # ---- execution ------------------------------------------------------
    def _status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _robot_xy(self):
        p = self._robot_pose()
        return p[:2] if p else None

    def _robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time()).transform
            q = t.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return t.translation.x, t.translation.y, yaw
        except Exception:
            return None

    def _run(self, name):
        try:
            ids = self.stations[name]
            if not self.nav.wait_for_server(timeout_sec=10.0):
                self._status(f'{name}: FAILED navigate_to_pose server unavailable')
                return
            pts = [self.nodes[i] for i in ids]
            labels = [f'node {i}' for i in ids]
            tols = [self.tol] * len(pts)
            if len(pts) >= 3:
                # "Out" of the station = from the inside node towards the middle of the mouth
                # (mean of entry and exit). Derived from the LIF nodes, not hard-coded.
                mx, my = (pts[0][0] + pts[-1][0]) / 2, (pts[0][1] + pts[-1][1]) / 2
                ox_, oy_ = mx - pts[1][0], my - pts[1][1]
                n_ = math.hypot(ox_, oy_) or 1.0
                ox_, oy_ = ox_ / n_, oy_ / n_
                pts = ([(pts[0][0] + ox_ * self.approach_dist, pts[0][1] + oy_ * self.approach_dist)] + pts
                       + [(pts[-1][0] + ox_ * self.exit_dist, pts[-1][1] + oy_ * self.exit_dist)])
                labels = ['approach'] + labels + ['exit-clear']
                tols = [self.staging_tol] + tols + [self.staging_tol]
            start = self._robot_xy() or pts[0]
            way_in = [math.atan2(y - py, x - px) for (px, py), (x, y) in zip([start] + pts[:-1], pts)]
            way_out = way_in[1:] + [way_in[-1]]
            yaws = [math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))
                    for a, b in zip(way_in, way_out)]
            for k, (lab, (x, y)) in enumerate(zip(labels, pts)):
                if not self._go(name, k + 1, len(pts), lab, x, y, yaws[k], tols[k]):
                    self._status(f'{name}: FAILED at waypoint {k + 1}/{len(pts)} ({lab})')
                    return
            self._status(f'{name}: COMPLETE ({len(pts)}/{len(pts)} waypoints, robot is out of the station)')
        finally:
            self._busy.release()

    def _go(self, name, k, n, node_id, x, y, yaw, tol):
        for attempt in range(1, self.max_retries + 1):
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = self.map_frame
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2)
            goal.pose.pose.orientation.w = math.cos(yaw / 2)
            self._status(f'{name}: waypoint {k}/{n} node {node_id} ({x:.3f},{y:.3f}) attempt {attempt}')

            done = threading.Event()
            result = {}
            fut = self.nav.send_goal_async(goal)
            fut.add_done_callback(lambda f: self._accepted(f, result, done))
            done.wait()
            if result.get('status') == 4:  # SUCCEEDED
                r = self._robot_pose()
                if r is None:
                    self.get_logger().warn('Nav2 succeeded but map->base_link unavailable; retrying')
                else:
                    d = math.hypot(r[0] - x, r[1] - y)
                    dyaw = abs(math.atan2(math.sin(r[2] - yaw), math.cos(r[2] - yaw)))
                    if d <= tol and dyaw <= self.yaw_tol:
                        self._status(f'{name}: reached waypoint {k}/{n} node {node_id} (err {d:.2f} m, {math.degrees(dyaw):.0f} deg)')
                        return True
                    self.get_logger().warn(f'Nav2 succeeded but robot {d:.2f} m / {math.degrees(dyaw):.0f} deg off; retrying')
            else:
                self.get_logger().warn(f'waypoint {k} attempt {attempt} failed (status {result.get("status")})')
            self._sleep(1.0)
        return False

    def _accepted(self, fut, result, done):
        handle = fut.result()
        if not handle.accepted:
            result['status'] = -1
            done.set()
            return
        handle.get_result_async().add_done_callback(
            lambda f: (result.update(status=f.result().status), done.set()))

    def _sleep(self, s):
        threading.Event().wait(s)


def main():
    rclpy.init()
    node = FleetManager()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
