#!/usr/bin/env python3
"""Multi-AMR fleet manager: every robot runs its own station mission at the same time, on a fixed node network.

Nodes / edges / stations / parking points come from config/fleet_graph.yaml (the red and blue points of the floor
picture). A robot only ever drives from node to node along an edge (a short straight leg, one Nav2
NavigateToPose goal per node) and a node counts as reached only when Nav2 SUCCEEDED and the measured
map->base_link pose (from /<ns>/tf) is within tolerance; otherwise the goal is re-sent.

Mission of every robot:  loading/unloading route -> its own station (stops 3-4 s at the centre node) ->
loading/unloading route again -> a parking point (deepest free one first, in order of arrival).
A station can be walked in either direction: the end nearer to the robot (by graph distance) is used.
Robots are launched `start_stagger` seconds apart and then all run simultaneously; nobody waits for anybody.

Traffic: every robot's lidar + collision monitor + costmaps treat the others as obstacles (Nav2). On top of that
this node knows every robot's pose and plan: when two robots meet head-on (or are deadlocked nose to nose)
ONE OF THEM, chosen at random, gives way: its goal is cancelled, it drives to a free pocket off the other
robot's path, waits there until the other has passed, and then re-sends the SAME node and carries on.

Services (std_srvs/Trigger):  /fms/fleet_mission (alias /fms/dual_mission)  all robots,  /fms/amrN_mission  one robot
Topic:  /fms/command (std_msgs/String) with the same names.   Status: /fms/status (std_msgs/String)
"""
import math
import os
import random
import threading
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav2_msgs.action import DriveOnHeading, NavigateToPose, Spin
from nav_msgs.msg import Path
from PIL import Image
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy import ndimage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer


class Agent:
    """One robot: its private TF tree, Nav2 action client, plan, and the yield state machine."""

    def __init__(self, fm, name, cfg):
        self.fm, self.name, self.cfg = fm, name, cfg
        self.home = (cfg['x'], cfg['y'], cfg['yaw'])   # spawn pose
        self.node = cfg['spawn']                        # graph node the robot stands at / reached last
        self.tasks = list(cfg.get('tasks', []))
        self.pinned = bool(self.tasks)   # tasks given in multi_robot.yaml are kept; otherwise drawn from task_pool
        self.tf = Buffer()
        self.active = False          # a Nav2 goal of this robot's mission is in flight
        self.mission = False         # this robot is part of a running mission
        self.yield_to = None         # Agent whose right of way we must respect (set by the traffic tick)
        self.yield_count = 0
        self.hold = False            # too close to another robot (<= stop_dist): standing still until nobody is near
        self.cooldown_until = 0.0
        self.plan = []               # last global plan from /<ns>/plan, [(x, y)]
        self.hist = deque(maxlen=12)  # (monotonic time, x, y) for the speed estimate
        cb = fm.cb
        tf_cb = MutuallyExclusiveCallbackGroup()   # keep this robot's TF messages in order
        static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        fm.create_subscription(TFMessage, f'/{name}/tf', lambda m: self._tf(m, False), 100, callback_group=tf_cb)
        fm.create_subscription(TFMessage, f'/{name}/tf_static', lambda m: self._tf(m, True), static_qos,
                               callback_group=tf_cb)
        fm.create_subscription(Path, f'/{name}/plan', self._on_plan, 5, callback_group=cb)
        self.nav = ActionClient(fm, NavigateToPose, f'/{name}/navigate_to_pose', callback_group=cb)
        self.drive = ActionClient(fm, DriveOnHeading, f'/{name}/drive_on_heading', callback_group=cb)
        self.spin = ActionClient(fm, Spin, f'/{name}/spin', callback_group=cb)

    # ---- state ------------------------------------------------------------------------------
    def _tf(self, msg, static):
        for t in msg.transforms:
            try:
                (self.tf.set_transform_static if static else self.tf.set_transform)(t, 'fms')
            except Exception:
                pass

    def _on_plan(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time()).transform
            q = t.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return t.translation.x, t.translation.y, yaw
        except Exception:
            return None

    def track(self):
        p = self.pose()
        if p:
            self.hist.append((time.monotonic(), p[0], p[1]))
        return p

    def velocity(self):
        """(vx, vy) in the map frame over the last ~1.5 s, or (0, 0)."""
        if len(self.hist) < 2:
            return 0.0, 0.0
        t1, x1, y1 = self.hist[-1]
        t0, x0, y0 = next((h for h in self.hist if t1 - h[0] <= 1.5), self.hist[0])
        dt = t1 - t0
        return ((x1 - x0) / dt, (y1 - y0) / dt) if dt > 0.2 else (0.0, 0.0)

    def speed(self):
        return math.hypot(*self.velocity())

    def busy(self):
        return self.active or self.speed() > 0.03

    def path_ahead(self, limit=3.0):
        """The part of this robot's global plan still in front of it (up to `limit` metres), or a short
        ray along its heading when no plan is available / it is not driving."""
        p = self.pose()
        if p is None:
            return []
        if self.active and len(self.plan) > 1:
            k = min(range(len(self.plan)), key=lambda i: math.hypot(self.plan[i][0] - p[0], self.plan[i][1] - p[1]))
            out, run = [], 0.0
            for a, b in zip(self.plan[k:], self.plan[k + 1:]):
                out.append(a)
                run += math.hypot(b[0] - a[0], b[1] - a[1])
                if run > limit:
                    break
            return out or [(p[0], p[1])]
        return [(p[0], p[1])]

    # ---- driving ----------------------------------------------------------------------------
    def go(self, tag, k, n, label, x, y, yaw, tol, early=0.0):
        """Drive to one waypoint. Re-sent (up to max_retries) until Nav2 succeeded AND the measured pose is
        within `tol`. A yield request pauses this call (goal cancelled, robot pulled aside) and re-sends
        the same waypoint afterwards without using up an attempt."""
        fm = self.fm
        attempt = 0
        while attempt < fm.max_retries:
            if self.yield_to is not None:
                self._yield()
            if self.hold:
                self._hold_wait()
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            fm.status(f'{self.name} {tag}: waypoint {k}/{n} {label} ({x:.3f},{y:.3f}) attempt {attempt + 1}')
            status, yielded = self._send(goal, early=early)
            if yielded:
                continue
            attempt += 1
            if status == 4:  # SUCCEEDED (or, for a transit node, close enough to hand over to the next node)
                r = self.pose()
                if r is None:
                    fm.get_logger().warn(f'{self.name}: Nav2 succeeded but map->base_link unavailable; retrying')
                else:
                    d = math.hypot(r[0] - x, r[1] - y)
                    if d <= (early + 0.03 if early else tol):
                        fm.status(f'{self.name} {tag}: reached waypoint {k}/{n} {label} (err {d:.2f} m)')
                        return True
                    fm.get_logger().warn(f'{self.name}: Nav2 succeeded but robot {d:.2f} m off; retrying')
            else:
                fm.get_logger().warn(f'{self.name}: waypoint {k} attempt {attempt} failed (status {status})')
            time.sleep(1.0)
        return False

    def settle(self, x, y, yaw):
        """Final touch at the start pose: Nav2 stops within its 7 cm goal tolerance, usually a few cm short and
        facing the way it arrived. Creep the remaining distance straight ahead (if the goal is in front and
        close) and spin back to the start heading. Best effort: the run is judged on the measured pose."""
        fm = self.fm
        p = self.pose()
        if p is None:
            return
        ahead = (x - p[0]) * math.cos(p[2]) + (y - p[1]) * math.sin(p[2])
        side = -(x - p[0]) * math.sin(p[2]) + (y - p[1]) * math.cos(p[2])
        if 0.02 < ahead < 0.15 and abs(side) < 0.06 and self.drive.wait_for_server(timeout_sec=2.0):
            g = DriveOnHeading.Goal()
            g.target = Point(x=ahead)
            g.speed = 0.05
            g.time_allowance.sec = 10
            self._simple(self.drive, g, 15.0)
        p = self.pose()
        if p is None:
            return
        delta = math.atan2(math.sin(yaw - p[2]), math.cos(yaw - p[2]))
        if abs(delta) > 0.25 and self.spin.wait_for_server(timeout_sec=2.0):
            g = Spin.Goal()
            g.target_yaw = delta
            g.time_allowance.sec = 40
            self._simple(self.spin, g, 45.0)
        p = self.pose()
        if p:
            fm.status(f'{self.name}: settled at ({p[0]:.3f},{p[1]:.3f}) heading {math.degrees(p[2]):.0f} deg '
                      f'(start pose was ({x:.3f},{y:.3f}) heading {math.degrees(yaw):.0f} deg)')

    def _simple(self, client, goal, timeout):
        done = threading.Event()

        def on_goal(f):
            h = f.result()
            if h.accepted:
                h.get_result_async().add_done_callback(lambda _: done.set())
            else:
                done.set()

        client.send_goal_async(goal).add_done_callback(on_goal)
        done.wait(timeout)

    def _send(self, goal, interruptible=True, early=0.0):
        """Send one goal; returns (status, yielded). Cancels it if a yield is requested meanwhile
        (never for the pull-over move itself, interruptible=False)."""
        done, res = threading.Event(), {}

        def on_result(f):
            res['status'] = f.result().status
            done.set()

        def on_goal(f):
            h = f.result()
            if not h.accepted:
                res['status'] = -1
                done.set()
                return
            res['handle'] = h
            h.get_result_async().add_done_callback(on_result)

        self.active = True
        self.nav.send_goal_async(goal).add_done_callback(on_goal)
        yielded = False
        gx, gy = goal.pose.pose.position.x, goal.pose.pose.position.y
        while not done.wait(0.1):
            if early and 'handle' in res:
                p = self.pose()
                if p and math.hypot(p[0] - gx, p[1] - gy) < early:
                    # transit node passed: send the next goal now instead of stopping here
                    res['handle'].cancel_goal_async()
                    done.wait(3.0)
                    self.active = False
                    return 4, False
            if interruptible and (self.yield_to is not None or self.hold):
                yielded = True
                deadline = time.monotonic() + 3.0
                while 'handle' not in res and not done.is_set() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if 'handle' in res:
                    res['handle'].cancel_goal_async()
                done.wait(5.0)
                break
        self.active = False
        return res.get('status', -1), yielded

    def _hold_wait(self):
        """Stand still (goal already cancelled) until no other robot is detected near, then carry on to the same node."""
        fm = self.fm
        t0, clear_since = time.monotonic(), None
        while time.monotonic() - t0 < fm.hold_timeout:
            if fm.robot_near(self, fm.release_dist) is None:
                clear_since = clear_since or time.monotonic()
                if time.monotonic() - clear_since > 0.5:
                    break
            else:
                clear_since = None
            time.sleep(0.1)
        else:
            fm.status(f'{self.name}: still blocked after {fm.hold_timeout:.0f} s - continuing anyway')
        fm.status(f'{self.name}: no robot detected - continuing to its node')
        self.hold = False
        self.cooldown_until = time.monotonic() + 3.0

    def _yield(self):
        """Give way: drive to a pocket off the other robot's path, wait until it has gone by."""
        fm, other = self.fm, self.yield_to
        self.yield_count += 1
        fm.status(f'{self.name}: YIELD - face-off with {other.name}, stopping and moving aside')
        pocket = fm.choose_pocket(self, other)
        if pocket is None:
            fm.status(f'{self.name}: no free pocket found - waiting in place')
        else:
            fm.status(f'{self.name}: pulling over to ({pocket[0]:.2f},{pocket[1]:.2f})')
            for _ in range(2):
                goal = NavigateToPose.Goal()
                goal.pose = PoseStamped()
                goal.pose.header.frame_id = 'map'
                goal.pose.pose.position.x, goal.pose.pose.position.y = pocket[0], pocket[1]
                goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(pocket[2] / 2), math.cos(pocket[2] / 2)
                status, _ = self._send(goal, interruptible=False)
                p = self.pose()
                if status == 4 or (p and math.hypot(p[0] - pocket[0], p[1] - pocket[1]) < 0.2):
                    break
        t0, clear_since = time.monotonic(), None
        while time.monotonic() - t0 < fm.yield_timeout:
            if fm.is_clear(self, other):
                clear_since = clear_since or time.monotonic()
                if time.monotonic() - clear_since > 1.0:
                    break
            else:
                clear_since = None
            time.sleep(0.2)
        else:
            fm.status(f'{self.name}: yield timed out after {fm.yield_timeout:.0f} s - resuming anyway')
        fm.status(f'{self.name}: path clear - resuming own route')
        self.yield_to = None
        self.cooldown_until = time.monotonic() + fm.yield_cooldown


class FleetManagerMulti(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        d = self.declare_parameter
        d('graph_file', '')          # config/fleet_graph.yaml
        d('robots_file', '')         # config/multi_robot.yaml
        d('map_yaml', '')
        d('keepout_yaml', '')        # human lanes: only used to keep pull-over pockets off them
        d('node_tolerance', 0.12)    # a node is reached within this distance of it (measured pose)
        d('home_tolerance', 0.12)
        d('max_retries', 5)
        d('pass_dist', 0.25)         # transit nodes count as passed within this distance (next goal is sent at once)
        d('start_stagger', 2.5)      # seconds between the launch of consecutive robots (they all keep running together)
        d('traffic_enabled', True)
        d('traffic_rate', 5.0)
        d('conflict_dist', 0.80)     # start looking at closing speed inside this centre distance
        d('hard_dist', 0.50)         # yield unconditionally (busy other robot) inside this distance
        d('closest_approach', 0.50)  # predicted closest approach that counts as a conflict
        d('resume_dist', 0.75)
        d('path_clearance', 0.55)    # pocket / resume: keep this far from the other robot's path ahead
        d('pocket_wall_clearance', 0.28)
        d('yield_timeout', 45.0)
        d('yield_cooldown', 6.0)
        d('stop_dist', 0.25)         # two robots this close (centre to centre): one of them stops
        d('release_dist', 0.40)      # ... and goes on when no other robot is within this distance
        d('hold_timeout', 20.0)
        g = self.get_parameter
        self.tol = g('node_tolerance').value
        self.home_tol = g('home_tolerance').value
        self.max_retries = g('max_retries').value
        for k in ('pass_dist', 'start_stagger', 'conflict_dist', 'hard_dist', 'closest_approach', 'resume_dist', 'path_clearance',
                  'pocket_wall_clearance', 'yield_timeout', 'yield_cooldown', 'stop_dist', 'release_dist', 'hold_timeout'):
            setattr(self, k, g(k).value)

        with open(g('graph_file').value) as f:
            gr = yaml.safe_load(f)
        self.nodes = {n: tuple(p) for n, p in gr['nodes'].items()}
        self.adj = {n: {} for n in self.nodes}
        edges = [tuple(e) for e in gr['edges']]
        self.stations = gr['stations']
        for st in self.stations.values():
            edges += list(zip(st, st[1:]))
        for a, b in edges:
            dist = math.hypot(self.nodes[a][0] - self.nodes[b][0], self.nodes[a][1] - self.nodes[b][1])
            self.adj[a][b] = self.adj[b][a] = dist
        self.parking = list(gr['parking'])
        self.lu = 'station_4'

        self._load_map(g('map_yaml').value)
        self.lane = self._load_lanes(g('keepout_yaml').value)
        self.cb = ReentrantCallbackGroup()
        with open(g('robots_file').value) as f:
            cfg = yaml.safe_load(f)
        self.task_pool = list(cfg.get('task_pool', []))
        self.dwell = tuple(cfg.get('station_dwell', [3.0, 4.0]))
        self._taken, self._slot_lock = set(), threading.Lock()
        self.agents = {n: Agent(self, n, c) for n, c in cfg['robots'].items()}
        self.order = list(self.agents.values())

        self.status_pub = self.create_publisher(String, '/fms/status', 10)
        self.create_subscription(String, '/fms/command', self._on_command, 10, callback_group=self.cb)
        for name in ['fleet_mission', 'dual_mission'] + [f'{n}_mission' for n in self.agents]:
            self.create_service(Trigger, f'/fms/{name}',
                                lambda req, res, name=name: self._on_service(name, res), callback_group=self.cb)
        self._busy = threading.Lock()
        self.min_sep = float('inf')
        self.yields = 0
        self._still = {}                 # (a, b) -> since when both have been standing still, close together
        if g('traffic_enabled').value:
            self.create_timer(1.0 / g('traffic_rate').value, self._traffic_tick, callback_group=self.cb)
        self.get_logger().info(f'FMS ready: robots={list(self.agents)} nodes={len(self.nodes)} '
                               f'stations={list(self.stations)} parking={self.parking}')

    # ---- map (for choosing pull-over pockets) -----------------------------------------------
    def _load_map(self, yaml_path):
        with open(yaml_path) as f:
            m = yaml.safe_load(f)
        img = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), m['image'])).convert('L'))
        self.res = float(m['resolution'])
        self.ox, self.oy = float(m['origin'][0]), float(m['origin'][1])
        free = img >= 0.75 * 255 if not m.get('negate', 0) else img <= 0.25 * 255
        self.clear = ndimage.distance_transform_edt(free) * self.res     # metres to nearest wall, per cell
        self.map_h = img.shape[0]

    def _load_lanes(self, yaml_path):
        if not yaml_path or not os.path.exists(yaml_path):
            return None
        with open(yaml_path) as f:
            m = yaml.safe_load(f)
        return np.array(Image.open(os.path.join(os.path.dirname(yaml_path), m['image'])).convert('L'))

    def _on_lane(self, x, y):
        if self.lane is None:
            return False
        c = int((x - self.ox) / self.res)
        r = self.map_h - 1 - int((y - self.oy) / self.res)
        return 0 <= r < self.lane.shape[0] and 0 <= c < self.lane.shape[1] and self.lane[r, c] > 0

    def _clearance(self, x, y):
        c = int((x - self.ox) / self.res)
        r = self.map_h - 1 - int((y - self.oy) / self.res)
        if 0 <= r < self.clear.shape[0] and 0 <= c < self.clear.shape[1]:
            return self.clear[r, c]
        return 0.0

    # ---- graph ------------------------------------------------------------------------------
    def route(self, src, dst):
        """Shortest node path src -> dst (Dijkstra), [src, ..., dst]; None if unreachable."""
        best, prev, todo = {src: 0.0}, {}, {src}
        while todo:
            u = min(todo, key=best.get)
            todo.discard(u)
            if u == dst:
                break
            for v, w in self.adj[u].items():
                if v not in best or best[u] + w < best[v] - 1e-9:
                    best[v], prev[v] = best[u] + w, u
                    todo.add(v)
        if dst not in best:
            return None
        path = [dst]
        while path[-1] != src:
            path.append(prev[path[-1]])
        return path[::-1]

    def cost(self, src, dst):
        r = self.route(src, dst)
        return float('inf') if r is None else sum(self.adj[a][b] for a, b in zip(r, r[1:]))

    # ---- triggers -----------------------------------------------------------------------------
    def _on_service(self, name, res):
        res.success, res.message = self._start(name)
        return res

    def _on_command(self, msg):
        self.get_logger().info(self._start(msg.data.strip())[1])

    def _start(self, name):
        names = {'fleet_mission', 'dual_mission'} | {f'{n}_mission' for n in self.agents}
        if name not in names:
            return False, f'unknown command "{name}"'
        if not self._busy.acquire(blocking=False):
            return False, 'FMS busy with another run'
        threading.Thread(target=self._run, args=(name,), daemon=True).start()
        return True, f'{name} started'

    def status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _run(self, name):
        try:
            self.min_sep, self.yields = float('inf'), 0
            self._taken = set()
            for a in self.agents.values():
                a.yield_count, a.cooldown_until, a.yield_to, a.hold = 0, 0.0, None, False
            fleet = name in ('fleet_mission', 'dual_mission')
            team = list(self.agents.values()) if fleet else [self.agents[name[:-len('_mission')]]]
            if fleet:
                self._assign_tasks()
            # launch order: the robot with the shortest way to the loading/unloading start goes first
            team.sort(key=lambda a: self.cost(a.node, self.stations[self.lu][0]))
            results = {}

            def worker(a, delay):
                a.mission = True
                try:
                    time.sleep(delay)
                    if delay:
                        self.status(f'{a.name}: leaving now ({delay:.1f} s after the first robot)')
                    results[a.name] = self._mission(a)
                except Exception as e:                       # never leave a robot thread silent
                    self.get_logger().error(f'{a.name}: mission crashed: {e}')
                    results[a.name] = False
                finally:
                    a.mission = False

            for a in team:
                a.mission = True                             # counted for the separation statistics from the start
            threads = [threading.Thread(target=worker, args=(a, i * self.start_stagger), daemon=True)
                       for i, a in enumerate(team)]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
            ok = all(results.values())
            sep = f'{self.min_sep:.2f} m' if self.min_sep != float('inf') else 'n/a'
            self.status(f'{name}: {"COMPLETE" if ok else "FAILED"} ' +
                        ' '.join(f'{n}={"ok" if r else "FAILED"}' for n, r in results.items()) +
                        f' | yields={sum(a.yield_count for a in self.agents.values())}'
                        f' | min robot-to-robot centre distance {sep} (contact below 0.21 m)')
        finally:
            self._busy.release()

    def _assign_tasks(self):
        """Every robot without pinned tasks gets a different station from the pool, at random."""
        free = [a for a in self.agents.values() if not a.pinned]
        taken = {t for a in self.agents.values() if a.pinned for t in a.tasks}
        pool = [t for t in self.task_pool if t not in taken]
        random.shuffle(pool)
        for a in free:
            a.tasks = [pool.pop()] if pool else []
        self.status('task assignment: ' + ', '.join(f'{a.name} -> {" ".join(a.tasks) or "nothing"}'
                                                   for a in self.agents.values()))

    # ---- mission ------------------------------------------------------------------------------
    def _mission(self, a):
        """loading/unloading -> own station(s) -> loading/unloading -> parking point."""
        for st in [self.lu] + a.tasks + [self.lu]:
            if not self._run_station(a, st):
                self.status(f'{a.name}: FAILED during {st}')
                return False
        return self._park(a)

    def _walk(self, a, path, tag, dwell_at=None, tol=None, early=0.0):
        """Drive the node list `path` in order (one Nav2 goal per node)."""
        n = len(path)
        for k, name in enumerate(path):
            x, y = self.nodes[name]
            if early and self.robot_at(a, x, y, 0.3):
                # a transit node another robot is standing on cannot be planned to (goal in its obstacle):
                # it would be passed without stopping anyway, so go straight on to the next node
                self.status(f'{a.name} {tag}: node {name} is occupied by another robot - passing it')
                a.node = name
                continue
            p = a.pose()
            yaw = math.atan2(y - p[1], x - p[0]) if p else 0.0
            if not a.go(tag, k + 1, n, f'node {name}', x, y, yaw, tol or self.tol, early=early):
                return False
            a.node = name
            if dwell_at is not None and name == dwell_at:
                t = random.uniform(*self.dwell)
                self.status(f'{a.name} {tag}: at the centre point ({name}) - standing still for {t:.1f} s')
                time.sleep(t)
        return True

    def _run_station(self, a, name):
        seq = self.stations[name]
        own = name != self.lu
        if own and self.cost(a.node, seq[-1]) < self.cost(a.node, seq[0]):
            seq = seq[::-1]                                  # enter at the end that is nearer to the robot
            self.status(f'{a.name} {name}: entering at node {seq[0]}, leaving at node {seq[-1]} '
                        f'(the end nearer to the robot)')
        path = self.route(a.node, seq[0])
        if path is None:
            self.status(f'{a.name} {name}: no route from node {a.node}')
            return False
        if not self._walk(a, path[1:-1], 'transit', early=self.pass_dist):          # the nodes between here and the station
            return False
        mid = seq[len(seq) // 2] if own else None
        if not self._walk(a, seq, name, dwell_at=mid):
            return False
        self.status(f'{a.name} {name}: COMPLETE (robot is out of the station at node {a.node})')
        return True

    def _near(self, a, node, d=0.15):
        p = a.pose()
        return p is not None and math.hypot(p[0] - self.nodes[node][0], p[1] - self.nodes[node][1]) < d

    def _park(self, a):
        """Take the deepest free parking point (first robot to arrive gets point 1, the next point 2, ...)."""
        with self._slot_lock:
            for name in self.parking:
                if name in self._taken:
                    continue
                self._taken.add(name)
                break
            else:
                self.status(f'{a.name}: no free parking point')
                return False
        self.status(f'{a.name}: takes parking point {name} at ({self.nodes[name][0]:.2f},{self.nodes[name][1]:.2f})')
        path = self.route(a.node, name)
        if path is None:
            return False
        if not (self._walk(a, path[1:-1], 'park', early=self.pass_dist) and self._walk(a, path[-1:], 'park')):
            self.status(f'{a.name}: FAILED reaching parking point {name}')
            return False
        x, y = self.nodes[name]
        a.settle(x, y, a.home[2])
        self.status(f'{a.name}: reached parking point {name}')
        return True

    # ---- traffic ------------------------------------------------------------------------------
    def _traffic_tick(self):
        poses = {a: a.track() for a in self.order}
        now = time.monotonic()
        for i, a in enumerate(self.order):
            for b in self.order[i + 1:]:
                pa, pb = poses[a], poses[b]
                if pa is None or pb is None:
                    continue
                d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
                if a.mission or b.mission:
                    self.min_sep = min(self.min_sep, d)
                if a.mission and b.mission and d <= self.stop_dist and not (a.hold or b.hold) \
                        and a.yield_to is None and b.yield_to is None:
                    # last-resort safety rule: too close - one of them stops, whether it is mid-goal, settling,
                    # or momentarily idle between goals (parking is exactly when both are near-stationary and
                    # this must still catch them)
                    candidates = [x for x in (a, b) if now >= x.cooldown_until]
                    if candidates:
                        moving = [x for x in candidates if x.speed() > 0.03]
                        victim = random.choice(moving or candidates)
                        other = b if victim is a else a
                        self.status(f'SAFETY: {a.name} and {b.name} are {d:.2f} m apart -> {victim.name} stops '
                                    f'until no robot is near ({other.name} carries on)')
                        victim.hold = True
                        continue
                if a.yield_to is not None or b.yield_to is not None:
                    continue                      # one of them is already giving way
                if not (a.busy() and b.busy()):
                    continue                      # the other one is parked: plain Nav2 obstacle avoidance
                why = self._conflict(a, b, pa, pb, d)
                if not why:
                    continue
                cands = [x for x in (a, b) if x.mission and x.active and now >= x.cooldown_until]
                if not cands:
                    continue
                victim = random.choice(cands)     # nobody has a fixed priority: either one may have to give way
                other = b if victim is a else a
                self.yields += 1
                self.status(f'TRAFFIC: {a.name} and {b.name} {why} (centre distance {d:.2f} m) '
                            f'-> {victim.name} gives way (picked at random)')
                victim.yield_to = other

    def _conflict(self, a, b, pa, pb, d):
        """Reason string if the pair is in a face-off (head-on / crossing / deadlock), else ''. Robots driving
        the same way or sideways are never a conflict."""
        now, key = time.monotonic(), (a.name, b.name)
        if a.speed() < 0.02 and b.speed() < 0.02 and d < 0.6 and a.active and b.active:
            since = self._still.setdefault(key, now)
            if now - since > 6.0:                # nose to nose and nobody moving: break the deadlock
                self._still.pop(key, None)
                return 'are deadlocked'
        else:
            self._still.pop(key, None)
        if d > self.conflict_dist:
            return ''
        if math.cos(pa[2] - pb[2]) > -0.5:       # only real face-offs (headings > 120 degrees apart) need a give-way;
            return ''                            # same-way / sideways traffic is left to Nav2's own obstacle avoidance
        rx, ry = pb[0] - pa[0], pb[1] - pa[1]
        vx = b.velocity()[0] - a.velocity()[0]
        vy = b.velocity()[1] - a.velocity()[1]
        closing = (rx * vx + ry * vy) / max(d, 1e-3)          # d(distance)/dt, negative = approaching
        if d < self.hard_dist and closing < -0.03:
            return 'are closing on each other'
        v2 = vx * vx + vy * vy
        if v2 < 1e-4:
            return ''
        t = max(0.0, min(4.0, -(rx * vx + ry * vy) / v2))
        if t > 0.0 and math.hypot(rx + vx * t, ry + vy * t) < self.closest_approach:
            return 'are on converging courses'
        return ''

    def robot_at(self, a, x, y, dist):
        """Another robot on a mission within `dist` of the map point (x, y)."""
        for o in self.agents.values():
            q = o.pose() if o is not a and o.mission else None
            if q and math.hypot(x - q[0], y - q[1]) <= dist:
                return o
        return None

    def robot_near(self, a, dist):
        """Another robot on a mission within `dist` (centre distance) of `a`, or None."""
        p = a.pose()
        if p is None:
            return None
        for o in self.agents.values():
            q = o.pose() if o is not a and o.mission else None
            if q and math.hypot(p[0] - q[0], p[1] - q[1]) <= dist:
                return o
        return None

    def is_clear(self, lo, hi):
        pl, ph = lo.pose(), hi.pose()
        if pl is None or ph is None:
            return True
        d = math.hypot(pl[0] - ph[0], pl[1] - ph[1])
        if not hi.busy():
            return d > 0.45
        ahead = hi.path_ahead()
        off_path = min(math.hypot(pl[0] - x, pl[1] - y) for x, y in ahead) if ahead else d
        return d >= self.resume_dist and off_path >= self.path_clearance

    def choose_pocket(self, lo, hi):
        """Free map spot near `lo`, off `hi`'s path and not reached by crossing `hi`: (x, y, yaw) or None."""
        pl, ph = lo.pose(), hi.pose()
        if pl is None or ph is None:
            return None
        ahead = hi.path_ahead(2.5)
        # densify the other robot's path so distances to it are meaningful
        line = []
        for a, b in zip(ahead, ahead[1:] + [ahead[-1]]):
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05))
            line += [(a[0] + (b[0] - a[0]) * s / n, a[1] + (b[1] - a[1]) * s / n) for s in range(n)]
        line.append(ph[:2])
        others = [q for q in (o.pose() for o in self.agents.values() if o is not lo and o is not hi) if q]
        for scale in (1.0, 0.8, 0.6):
            best = None
            r = 1.6
            for gx in np.arange(pl[0] - r, pl[0] + r + 1e-6, 0.1):
                for gy in np.arange(pl[1] - r, pl[1] + r + 1e-6, 0.1):
                    if self._clearance(gx, gy) < self.pocket_wall_clearance * scale:
                        continue
                    if self._on_lane(gx, gy):
                        continue                  # never park on a human lane
                    if any(math.hypot(gx - q[0], gy - q[1]) < 0.45 for q in others):
                        continue                  # another robot is there
                    if math.hypot(gx - ph[0], gy - ph[1]) < (self.resume_dist - 0.05) * scale:
                        continue
                    if min(math.hypot(gx - x, gy - y) for x, y in line) < self.path_clearance * scale:
                        continue
                    if self._seg_dist(pl[:2], (gx, gy), ph[:2]) < 0.40 * scale:
                        continue                  # would have to drive across the other robot
                    cost = math.hypot(gx - pl[0], gy - pl[1])
                    if best is None or cost < best[0]:
                        best = (cost, gx, gy)
            if best:
                return best[1], best[2], math.atan2(best[2] - pl[1], best[1] - pl[0])
        return None

    @staticmethod
    def _seg_dist(a, b, p):
        ax, ay, bx, by = a[0], a[1], b[0], b[1]
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
        return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def main():
    rclpy.init()
    node = FleetManagerMulti()
    ex = MultiThreadedExecutor(num_threads=12)
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
