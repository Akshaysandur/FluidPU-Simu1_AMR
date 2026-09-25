# Architecture

## The base_footprint / base_link split — read this first

`amr.urdf.xacro` accepts `use_webots:=true|false` and this ONE flag
changes the root of the entire kinematic tree:

- `use_webots:=false` (RViz-only / `display.launch.py` style use): root is
  `base_footprint`, with a fixed joint up to `base_link` (matches normal
  ROS convention — see the `kdl_parser` warning below for why this
  convention exists).
- `use_webots:=true` (everything under `amr_simulation`): `base_footprint`
  is compiled out entirely (`base.urdf.xacro` wraps it in
  `<xacro:unless value="$(arg use_webots)">`). **`base_link` is the true
  root.**

This was forced by how Webots physics works: when `base_footprint` (a
massless placeholder) is the root, `urdf2webots` nests `base_link`'s mass
and collision inside a child `Solid`. That placement is *supposed* to be
valid per Webots' own rules, but empirically it did not let gravity act on
the robot. Making `base_link` the literal root causes `urdf2webots` to put
`boundingObject`/`physics` directly on the `Robot` node itself instead —
and that is the configuration that was confirmed (by direct visual
observation in Webots, not just log inspection) to actually work.

**Consequence — this frame split touches far more files than you'd
expect. Every one of these must say `base_link`, not `base_footprint`,
when running under `amr_simulation`:**

| File | Parameter |
|---|---|
| `amr_description/config/amr/controllers.yaml` | `base_frame_id` |
| `amr_simulation/config/ekf.yaml` | `base_link_frame` |
| `amr_navigation/config/amcl.yaml` | `base_frame_id` |
| `amr_description/rviz/viewport_config.rviz` | `Global Options: Fixed Frame` |

If you add a new node that needs a base frame, it goes here too.

**Side effect to know about, not fix:** `robot_state_publisher` logs
`WARN [kdl_parser]: The root link base_link has an inertia specified in
the URDF, but KDL does not support a root link with an inertia.` on every
launch under `use_webots:=true`. This is expected and harmless — it's
exactly the tradeoff described above, not a new bug.

## TF tree (under use_webots:=true)

```
map                              (published by AMCL or slam_toolbox — pick one, not both)
 └─ odom                         (published by ekf_node — see below)
     └─ base_link                (root; robot_state_publisher publishes everything below via fixed/hinge joints)
         ├─ front_left_wheel_link / front_right_wheel_link / rear_left_wheel_link / rear_right_wheel_link
         ├─ laser_frame          (Lidar)
         ├─ imu_link             (IMU triplet)
         └─ cam_1_link
             ├─ cam_1_color_frame / cam_1_color_optical_frame
             ├─ cam_1_depth_frame / cam_1_depth_optical_frame
             └─ cam_1_infra1_frame / cam_1_infra2_frame (+ optical variants)
```

**Who publishes `odom -> base_link`:** `ekf_node` (robot_localization),
and ONLY `ekf_node`. The installed `mecanum_drive_controller`
(ros-humble-mecanum-drive-controller 2.53.3) has an `enable_odom_tf`
parameter but **no `tf2_ros::TransformBroadcaster` anywhere in its
compiled library** — confirmed via `strings` on `libmecanum_drive_controller.so`.
Its TF-shaped data only reaches the topic
`/mecanum_drive_controller/tf_odometry`, which nothing bridges onto the
real `/tf` tree. `enable_odom_tf` is left `false` in `controllers.yaml`
for this reason — turning it on does nothing useful on this build.

**Who publishes `map -> odom`:** either `slam_toolbox`
(`mapping.launch.py`, mapping mode) or `nav2_amcl`
(`amr_navigation/localization.launch.py`). **Never run both at once** —
they'll fight over the same transform.

## Node / data flow

```
Webots (webots-bin)
  │  IPC (port 1234)
  ▼
WebotsController (webots_ros2_driver)
  │  loads controllers.yaml, spawns ros2_control hardware interface
  ▼
controller_manager
  ├─ joint_state_broadcaster  → /joint_states
  └─ mecanum_drive_controller → /mecanum_drive_controller/odometry (nav_msgs/Odometry)
                              → subscribes /mecanum_drive_controller/reference_unstamped (geometry_msgs/Twist)
                                 (NOT /cmd_vel — see decisions-and-gotchas.md)

twist_mux (Multiplexer)
  in:  /cmd_vel_nav (Nav2, priority 10), /cmd_vel_teleop (Keyboard, priority 100)
  out: /mecanum_drive_controller/reference_unstamped

Native Webots devices (via <webots> block in sim_control.urdf.xacro):
  Lidar        → /scan (LaserScan), /scan/point_cloud (PointCloud2)
  IMU triplet  → /imu/data (sensor_msgs/Imu, combined by webots_ros2_driver::Ros2IMU)
  Camera+Depth → /camera/rgbd (webots_ros2_driver::Ros2RGBD) and per-device
                 topics under /amr_robot/camera/*, /amr_robot/range_finder/*

ekf_node (robot_localization)
  in:  /mecanum_drive_controller/odometry, /imu/data
  out: odom -> base_link TF, /odometry/filtered

slam_toolbox (mapping.launch.py) OR nav2_amcl (amr_navigation)
  in:  /scan, odom -> base_link TF
  out: map -> odom TF, /map
```

## Topic reference

| Topic | Type | Publisher |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Lidar static plugin |
| `/scan/point_cloud` | `sensor_msgs/PointCloud2` | Lidar static plugin (separate topic — see gotchas) |
| `/imu/data` | `sensor_msgs/Imu` | `webots_ros2_driver::Ros2IMU` |
| `/camera/rgbd` | `webots_ros2_driver::Ros2RGBD` combined output | |
| `/mecanum_drive_controller/odometry` | `nav_msgs/Odometry` | `mecanum_drive_controller` |
| `/mecanum_drive_controller/reference_unstamped` | `geometry_msgs/Twist` | (subscribed, not published) — drive commands go here |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` or `nav2_map_server` |
