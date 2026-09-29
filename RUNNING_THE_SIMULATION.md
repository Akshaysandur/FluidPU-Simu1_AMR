# Running, Monitoring and Commanding the AMR Simulation

TurtleBot3 Burger in Webots R2025a + ROS 2 Humble + Nav2. The robot drives autonomously through the
lab's stations (enter -> inside -> exit) using the LIF/GeoJSON graph, closed-loop with AMCL, odometry,
lidar and Nav2 costmaps. All paths below assume the workspace is `/home/akshay/rosws`.

---

## 1. The 30-second version

| I want to... | Command |
|---|---|
| Run the **whole multi-station mission** (fresh restart) | `bash /home/akshay/rosws/tools/boot_mission.sh` |
| Run **Station 1 only** (fresh restart) | `bash /home/akshay/rosws/tools/boot_sim.sh` |
| **Stop everything** | `bash /home/akshay/rosws/tools/stop_sim.sh` |
| Send a command while things are already running | `ros2 service call /fms/full_mission std_srvs/srv/Trigger` (after `source tools/env.sh`) |
| Watch progress | `tail -f /tmp/amr_logs/fms.log` or `ros2 topic echo /fms/status` |

Both boot scripts take ~2-5 minutes and print the result at the end. Logs go to `/tmp/amr_logs/`.

---

## 2. What is running (architecture in one screen)

```
Webots world (simulation.wbt) + TurtleBot3 Burger        <- sim.launch.py   (amr_simulation)
   | /scan (lidar), wheel odometry, /clock, TF odom->base_link
   v
Nav2 stack                                               <- navigation.launch.py (amr_navigation)
   map_server + AMCL (localisation, map->odom)           <- lifecycle_manager_localization
   planner_server (SmacPlanner2D), controller_server (Regulated Pure Pursuit),
   behavior_server (recoveries), bt_navigator, velocity_smoother,
   collision_monitor (last-line wall stop)               <- lifecycle_manager_navigation (starts ~12 s later)
   |  action: /navigate_to_pose
   v
Fleet manager (station sequencer)                        <- fleet_control.launch.py (scripts/fleet_manager.py)
   reads config/lab_graph.json (LIF nodes) -> converts to map frame -> sends one waypoint at a time
```

Key facts:
- **Map frame == Webots world frame.** The map is generated from the world file (`maps/room_map_world.*`).
- **LIF -> map:** `map = (1.51 - lif_x, 1.25 - lif_y)` (180 deg rotation, scale 1).
- Robot start pose (and AMCL initial pose): `x=1.27, y=0.38, yaw=pi` (just below the orange start box).
- Speeds: max 0.16-0.18 m/s linear, 0.9 rad/s angular; collision monitor stops ~2 cm from a wall.
- The workspace must be sourced **after** the ROS underlay (a patched `tf2_ros` in this workspace fixes a
  deadlock that otherwise makes goals "succeed" instantly). `tools/env.sh` does this for you.

---

## 3. Every command, in order (what to run and why)

### 3.1 One-time environment for any terminal
```bash
source /home/akshay/rosws/tools/env.sh
```
Sets Cyclone DDS to loopback-only (`CYCLONEDDS_URI`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`), sources
`~/ros2_humble` then `/home/akshay/rosws/install`, and defines `LOGS=/tmp/amr_logs`.
**Every terminal that uses `ros2` must do this**, otherwise it will not see the running nodes.

### 3.2 Automatic way (recommended)
```bash
bash /home/akshay/rosws/tools/boot_mission.sh     # full mission
# or
bash /home/akshay/rosws/tools/boot_sim.sh         # station 1 only
```
What the script does (so you know what to do by hand if a step fails):
1. `stop_sim.sh` - kills any leftover simulation processes.
2. `ros2 launch amr_simulation sim.launch.py` - starts Webots, the robot driver, RViz; waits 45 s.
3. `ros2 launch amr_navigation navigation.launch.py` - starts Nav2; waits until the log shows
   "Managed nodes are active" **twice** (localisation group, then navigation group).
4. `ros2 launch amr_navigation fleet_control.launch.py` - starts the station sequencer; waits 8 s.
5. Calls the trigger service (`/fms/full_mission` or `/fms/station_1`) and polls `/tmp/amr_logs/fms.log`
   until `COMPLETE` or `FAILED`, then prints the `reached / COMPLETE / FAILED` lines.

Tip: run it detached so closing the terminal does not kill it:
```bash
setsid nohup bash /home/akshay/rosws/tools/boot_mission.sh > /tmp/mission_out.log 2>&1 < /dev/null &
tail -f /tmp/mission_out.log
```

### 3.3 Manual way (three terminals, each starts with `source tools/env.sh`)
```bash
# Terminal 1 - simulation (wait ~45 s until the Webots window shows the robot)
ros2 launch amr_simulation sim.launch.py

# Terminal 2 - Nav2 (wait for "Managed nodes are active" to appear twice)
ros2 launch amr_navigation navigation.launch.py

# Terminal 3 - station sequencer
ros2 launch amr_navigation fleet_control.launch.py
```
Then send a command (section 4).

### 3.4 Stopping
```bash
bash /home/akshay/rosws/tools/stop_sim.sh      # prints "simulation stopped"
```
Do **not** use `pkill -f <pattern>` inside scripts - it can match and kill its own shell. `stop_sim.sh`
filters by PID instead.

---

## 4. Sending commands (how to make it go station 1 -> station 2 -> ...)

The fleet manager exposes one `std_srvs/Trigger` service per route, plus a topic:

| Service | Route (LIF node ids) | What the robot does |
|---|---|---|
| `/fms/station_1` | 18 -> 10 -> 17 | top-middle station: enter, inside, exit, clear |
| `/fms/station_2` | 14 -> 12 -> 13 | top-left station: enter, inside, exit, clear |
| `/fms/station_3` | 16 -> 11 -> 15 | bottom-left station: enter, inside, exit, clear |
| `/fms/station_4` | 21 -> 9 -> 20 -> 8 -> 19 | bottom bay: unloading entry, inside, middle (20), loading (8), exit to lane |
| `/fms/full_mission` | station_1, 2, 3, 4, then home | the whole run in one go, ends at the start pose (1.27, 0.38) |

Send one (after `source tools/env.sh`):
```bash
ros2 service call /fms/station_1 std_srvs/srv/Trigger
ros2 service call /fms/station_2 std_srvs/srv/Trigger
ros2 service call /fms/full_mission std_srvs/srv/Trigger
```
The reply `success=True, message='<name> started'` only means the run **started**. Completion is reported
on `/fms/status` and in the log (section 5).

Or through the topic (same effect):
```bash
ros2 topic pub --once /fms/command std_msgs/msg/String "{data: 'station_2'}"
```

Rules:
- Runs are **not** queued: a second command while one is running returns `FMS busy with another run`.
- Each run starts from **wherever the robot currently is**: the first waypoint is an "approach" point in
  front of the station entry, so you can chain manually: `station_1` (wait for COMPLETE) -> `station_2` ->
  `station_3` -> `station_4`. `full_mission` just does this for you and then drives home.
- The robot must be localised (AMCL running, start pose correct) before you send a command.

### What one station run consists of
`approach point (0.30 m before entry)` -> `entry node` -> `inside node` -> `exit node` ->
`exit-clear point (0.35 m beyond exit)`. `station_4` has 7 waypoints (approach, 21, 9, 20, 8, 19,
exit-clear). A waypoint counts as reached only if Nav2 reports SUCCEEDED **and** the measured
map->base_link pose is within tolerance (0.10 m for nodes, 0.15 m for approach/exit-clear). On a miss it
retries (max 5) and never skips a waypoint. If it still fails you get `FAILED at waypoint k/n`.

---

## 5. Monitoring

### 5.1 Mission progress (the main one)
```bash
tail -f /tmp/amr_logs/fms.log | grep --line-buffered -E "waypoint|COMPLETE|FAILED|failed"
# or live from ROS:
ros2 topic echo /fms/status
```
Expected lines:
```
station_1: waypoint 1/5 node approach (0.386,0.381) attempt 1
station_1: reached waypoint 1/5 node approach (err 0.07 m, 45 deg)
...
station_1: COMPLETE (5/5 waypoints, robot is out of the station)
full_mission: COMPLETE (4 stations + home, robot is back at the start)
```
`err` is the distance to the waypoint in metres (0.05-0.08 m is normal). The "deg" value is heading
difference and is informational only - goals are position-only.

### 5.2 Nav2 / system health
```bash
tail -f /tmp/amr_logs/nav.log          # planner/controller/AMCL messages
tail -f /tmp/amr_logs/sim.log          # Webots + driver output
ros2 node list                         # planner_server, controller_server, amcl, fleet_manager, ...
ros2 lifecycle get /controller_server  # should be "active"
```

### 5.3 Where is the robot?
```bash
ros2 run tf2_ros tf2_echo map base_link          # pose in the map (== Webots world) frame
ros2 topic echo /odom --once                     # wheel odometry
ros2 topic echo /amcl_pose --once                # localisation estimate
```

### 5.4 Motion and safety
```bash
ros2 topic echo /cmd_vel                         # final command to the wheels
ros2 topic hz /scan                              # lidar alive?
ros2 topic echo /collision_monitor_state --once  # safety stop state (if published)
```

### 5.5 Visualisation
RViz is launched by `sim.launch.py`; if it cannot open (X/GL error `Invalid parentWindowHandle`) the run
is unaffected. Start it manually from a graphical terminal:
`rviz2 -d $(ros2 pkg prefix amr_description)/share/amr_description/rviz/viewport_config.rviz`.
Useful displays: map, `/scan`, global plan, local costmap, robot footprint.

---

## 6. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Goal "succeeds" instantly, robot does not move | tf2 deadlock. Make sure the workspace was sourced **after** the underlay (`source tools/env.sh`). Check `ros2 topic echo /local_costmap/published_footprint`: the header stamp must keep advancing. Restart via `stop_sim.sh` + boot script. |
| `Nav2 did not come up - see /tmp/amr_logs/nav.log` | Simulation not ready or stale processes. Run `stop_sim.sh`, boot again; check `sim.log` for errors. |
| `ros2 service call` hangs / `/fms/...` not found | Terminal missing `source tools/env.sh`, or fleet manager not running (`ros2 node list`). |
| `FMS busy with another run` | A run is still going. Wait for COMPLETE/FAILED. |
| `FAILED at waypoint k/n` | Look at `fms.log` warnings (`attempt N failed`, `robot X m off`) and `nav.log`. Usually the robot was mis-localised: reboot with `boot_*.sh` so it starts from the spawn pose. |
| Robot starts from a non-default place | AMCL initial pose is fixed to (1.27, 0.38, pi). Only reset the world/simulation to the spawn pose, or set a new 2D Pose Estimate in RViz. |
| RViz window error on launch | Harmless for the run (see 5.5). |
| Webots closed/crashed mid-run | Run `stop_sim.sh`, then boot again. |
| Stale DDS state after a crash | `stop_sim.sh` also clears `/dev/shm/fastrtps_*`; boot again. |

---

## 7. Adding or changing a route

Edit `src/amr_navigation/config/fleet_manager.yaml`:
```yaml
station_names: ["station_1", "station_2", "station_3", "station_4", "station_5"]
station_5_nodes: [<entry id>, <inside id>, <exit id>]      # LIF node ids from config/lab_graph.json
mission_stations: [...]                                     # (optional) what full_mission chains
```
Rules: 3 nodes = enter/inside/exit (approach and exit-clear directions are derived from the geometry);
more than 3 nodes = approach straight back along the first leg and exit-clear straight on along the last
leg. Then rebuild/copy the config (`colcon build --packages-select amr_navigation`, or copy the yaml into
`install/amr_navigation/share/amr_navigation/config/`), restart only the fleet manager
(`ros2 launch amr_navigation fleet_control.launch.py`) and call `/fms/station_5`.

Node ids -> positions: `config/lab_graph.json` holds the LIF points; map coordinates =
`(1.51 - x, 1.25 - y)`. The fleet manager log prints the computed map coordinates of every waypoint.

---

## 8. File map

| Path | Purpose |
|---|---|
| `tools/env.sh` | DDS + ROS environment (source in every terminal) |
| `tools/boot_sim.sh` | fresh boot + Station 1 once ("boot up") |
| `tools/boot_mission.sh` | fresh boot + full mission ("boot multi") |
| `tools/stop_sim.sh` | stop all simulation processes |
| `src/amr_navigation/scripts/fleet_manager.py` | station sequencer (services `/fms/*`) |
| `src/amr_navigation/config/fleet_manager.yaml` | station routes, tolerances, LIF->map transform |
| `src/amr_navigation/config/lab_graph.json` | the LIF/GeoJSON graph (22 nodes) |
| `src/amr_navigation/config/nav2_params.yaml` | Nav2 tuning (RPP controller, costmaps, collision monitor) |
| `src/amr_navigation/config/amcl.yaml` | localisation + initial pose |
| `src/amr_navigation/maps/` | world-derived map + generator `make_world_map.py` |
| `src/project_context/decisions-and-gotchas.md` | design decisions and the debugging history |
| `/tmp/amr_logs/{sim,nav,fms}.log` | runtime logs (recreated on every boot) |

---

## 9. Known limits
- Wall clearance (2-4 cm at the closest) was measured on Station 1 only.
- Tested against walls and pillars, not against moving or newly placed obstacles.
- Wheel odometry is used (the RF2O -> EKF chain is not restored).
- Station 4's node mapping was matched by position from a drawn route, not from LIF station names.
