# Decisions and Gotchas

Each entry: what was done, why, and where it will bite you if forgotten.
All of these were verified against real installed source (`strings` on
compiled `.so`/binaries, real header files, or direct empirical testing
against the real Webots node schemas) — not assumed from tutorials, which
is exactly why several of them contradict what a generic ROS2/Nav2
tutorial would tell you to do.

## Controller stack

**`mecanum_drive_controller` is a ChainableController, not the classic
tutorial-era controller.** The installed version
(`ros-humble-mecanum-drive-controller` 2.53.3) has a completely different
parameter schema than most online examples: `front_left_wheel_command_joint_name`
/ `..._state_joint_name` per wheel, `kinematics.wheels_radius`,
`kinematics.sum_of_robot_center_projection_on_X_Y_axis` (= lx + ly from the
wheel geometry), not the old flat `wheel_base`/`wheel_separation`/`wheel_radius`.
Get the real schema from `/opt/ros/humble/include/mecanum_drive_controller/mecanum_drive_controller_parameters.hpp`
if it ever needs revisiting.

**It does not subscribe to `/cmd_vel`.** Commands go to
`<controller_name>/reference_unstamped` (`geometry_msgs/Twist`, since
`use_stamped_vel: false`) or `.../reference` (`TwistStamped`) if that flag
is `true`. `teleop.launch.py` and the navigation stack's velocity output
must target `/mecanum_drive_controller/reference_unstamped`.

**It cannot publish TF**, regardless of `enable_odom_tf`. See
architecture.md's TF section. `enable_odom_tf: false` is deliberate, not
an oversight.

**Controller update-rate/Webots timestep mismatch is a real but harmless
warning.** `update_rate: 50` (20ms) vs. Webots' 32ms `basicTimeStep`
triggers `Ros2Control.cpp`'s own `RCLCPP_WARN_STREAM` every launch. It's
cosmetic — confirmed in source it never blocks or delays anything.

**Mecanum physics requires mirrored asymmetric friction.** 
Applying a single `<contactMaterial>` with a 45-degree rotation to all four wheels collapses the mecanum force decomposition (all diagonal slip vectors point the same way), causing the robot to "ice skate" diagonally instead of rotating in place.
Fixed by creating two Webots `ContactProperties` blocks (`wheel_pos_45` at `+0.785398` rad, and `wheel_neg_45` at `-0.785398` rad) and mapping FL/RR to the positive material and FR/RL to the negative material. This relies on `fix_proto.py` to inject the `contactMaterial` fields into the raw PROTO file, because `urdf2webots` ignores them.

**Starting torque stiction stalls.**
High static URDF `<dynamics damping="0.1" friction="0.05"/>` stalled the simulated motors. Reduced to `0.01` and added explicit `<limit effort="30.0" velocity="15.0"/>` to `wheel.urdf.xacro` to un-stall the PID. Reduced `coulombFriction` in `simulation.wbt` from 3.5 to 0.85 to stop wheel dragging.

**Twist Multiplexing.**
Multiple nodes (`teleop`, `nav2`, `lidar_docker.py`) command velocity. They all map to `/cmd_vel_<source>` which `twist_mux` prioritizes and forwards to `/mecanum_drive_controller/reference_unstamped`. Do NOT command the controller topic directly anymore.

## EKF (robot_localization)

**`ekf_node` is the sole `odom -> base_link` TF publisher** (see
architecture.md). `publish_tf: true` here matters a lot.

**`predict_to_current_time: true` crashes `ekf_node` (SIGSEGV) under live
sensor data**, on the installed version (robot_localization 3.5.4-1jammy).
Confirmed it does NOT crash standalone with zero sensor input (ran fine
15s under `gdb` with no publishers), so it's specifically the live
prediction path that's broken. Currently commented out in `ekf.yaml`. If
you want this feature, test it in isolation first, ideally after an apt
upgrade to see if a newer point release fixes it. A later task asked to
re-enable it (to fix a supposed TF extrapolation race with AMCL on boot)
and was declined for this reason — re-verified against this same evidence
rather than re-tested live, since the crash was already reproduced once.

**`transform_time_offset` is not a real settable parameter on this
installed version.** A task asked to add `transform_time_offset: 0.1` to
future-date the published TF. Checked `ros_filter.hpp`: there's an
internal `tf_time_offset_` member, but it's computed internally, not
exposed as a ROS parameter — confirmed by `strings` on the actual
`ekf_node` binary, which contains zero occurrences of `offset` anywhere.
Setting it in `ekf.yaml` would not error, it would just be silently
ignored (undeclared parameter override), so it can't provide the effect
it was requested for. Not applied.

## AMCL

**`base_frame_id` must be `base_link`**, not the tutorial-default
`base_footprint` (see architecture.md).

**`robot_model_type` must be explicitly set to `"nav2_amcl::OmniMotionModel"`.**
The default is `nav2_amcl::DifferentialMotionModel`, which assumes
forward/rotate-only motion and cannot model this robot's mecanum strafing
at all. Confirmed `OmniMotionModel` is a real registered plugin via
`/opt/ros/humble/share/nav2_amcl/plugins.xml`.

**`set_initial_pose: true` is REQUIRED alongside the `initial_pose:` block**
for AMCL to actually use it on startup. This is easy to miss: AMCL's own
parameter description string (visible via `strings` on `libamcl_core.so`)
says the `initial_pose*` parameters only take effect "with parameter
`set_initial_pose: true`" — without that flag they're silently ignored and
AMCL still waits for a manual "2D Pose Estimate" in RViz.

**`initial_pose` is expressed in the MAP frame, not Webots' world frame —
these are two different coordinate systems and are not expected to
numerically match.** `simulation.wbt`'s `amr_robot` spawn `translation`/
`rotation` (`1.27 0.38 0.104`, yaw `3.14159`) is where the robot boots up
in Webots' world coordinates; `amcl.yaml`'s `initial_pose`
(`x: -0.030, y: 0.357, z: 0.0, yaw: -0.027`) is the verified pose of that
same boot-time position relative to `room_map`'s origin. Don't try to make
the two match numerically — if the robot's real position relative to the
saved map ever changes (new spawn point, new map, hand-moved robot before
boot), re-verify and update `initial_pose` independently of
`simulation.wbt`.

**`transform_tolerance` raised from `1.0` to `1.5`** to widen AMCL's
allowed TF lookup delay window on boot, as part of the same "TF
extrapolation race with EKF on boot" task above (the `ekf.yaml` half of
that task was declined — see EKF section — but this half is a real,
already-used AMCL parameter and a safe, low-risk widening on its own).

## Nav2 stack (Milestone 3: costmaps, planner, controller, behaviors, BT)

**Several plugin/parameter names in the original task spec were wrong for
this installed version (Nav2 1.1.20, Humble) — all verified by `strings`
on the real compiled `.so`/binaries and the real `plugins.xml` files
before writing `nav2_params.yaml`, not assumed:**

- `nav2_controller`'s progress/goal checkers register only a `type=`
  (C++ namespace form, `nav2_controller::SimpleProgressChecker` /
  `nav2_controller::SimpleGoalChecker`) — no `name=` alias. The slash form
  (`nav2_controller/SimpleProgressChecker`) does not exist and fails to load.
- DWB's `StandardTrajectoryGenerator` sample-count parameters are
  `vx_samples` / `vy_samples` / `vtheta_samples` (confirmed via `strings`
  on `libstandard_traj_generator.so`) — **not** `vel_xs_samples` /
  `vel_ys_samples` / `vel_thetas_samples`. The wrong names would have been
  silently ignored (undeclared parameter override, no error), leaving DWB
  on its own default sample counts and never actually widening Y-velocity
  sampling — directly defeating the point of tuning DWB for mecanum strafing.
- `nav2_behaviors`'s registered plugin name is `nav2_behaviors/BackUp`
  (capital U — confirmed via `behavior_plugin.xml`), not `.../Backup`.
- `behavior_server` has no `local_frame` parameter at all (confirmed via
  `strings` on `libbehavior_server_core.so` — only `global_frame` exists).
  It was also set to `map` in the task spec; changed to `odom`, since
  Spin/BackUp execute short safety-critical motions against the local
  costmap/odometry (the same frame `local_costmap` itself operates in),
  not the slower map-frame AMCL estimate.
- `bt_navigator` has no `odom_frame` parameter (confirmed via `strings` on
  `libbt_navigator_core.so` — only `odom_topic`, a topic name not a TF
  frame, exists). Set to `/odometry/filtered` (the EKF-fused output — see
  architecture.md's node/data flow), not the raw
  `/mecanum_drive_controller/odometry`.
- `bt_navigator`'s real parameter for the default behavior tree file is
  `default_nav_to_pose_bt_xml`, not `default_bt_xml_filename` (also
  confirmed via `strings`). Left unset entirely in `nav2_params.yaml`:
  the requested file (`navigate_to_pose_w_replanning_and_recovery.xml`)
  is already `bt_navigator`'s own built-in default when this parameter is
  absent, so setting a wrong-named key would have been a silent no-op
  anyway rather than an active fix.

**`nav2_params.yaml`'s `amcl:` block intentionally duplicates the
standalone `amcl.yaml`, in full** — the task that created this file
listed only `set_initial_pose`/`initial_pose`/`transform_tolerance` there,
which (if used as written) would have silently dropped
`robot_model_type: nav2_amcl::OmniMotionModel` and reverted AMCL to the
default `DifferentialMotionModel` for the whole Nav2-stack launch path,
reintroducing the exact bug fixed in Milestone 2. If AMCL tuning changes,
update both `amcl.yaml` and `nav2_params.yaml`'s `amcl:` block.

**`controller_server` always publishes on `cmd_vel`, remapped in
`navigation.launch.py`** to `/mecanum_drive_controller/reference_unstamped`
— the same remap teleop needs, for the same reason (see Controller stack
section above). Without it Nav2 computes velocities that never reach the
wheels.

**`local_costmap`'s `width`/`height` must be plain integers, not
decimals.** `Costmap2DROS` pre-declares these as int-typed parameters in
its C++ constructor; ROS 2's strict parameter typing then rejects
overwriting an already-declared `int` with a `double` at load time.
Reproduced live: `width: 2.5` / `height: 2.5` crashed `controller_server`
on launch (`InvalidParameterTypeException`, "is of type {integer}, setting
it to {double} is not allowed") — the exact same bug class as the earlier
AMCL `initial_pose` crash, just inverted (there the fix was decimals;
here it's whole numbers). Confirmed against the real
`webots_ros2_turtlebot` reference config, which also uses plain integers
for these two fields. Fixed to `width: 3` / `height: 3` — still divided
internally by `resolution` to get cell counts, so nothing is lost.

**`behavior_server` needs a `"wait"` plugin even if nothing calls it
directly.** `bt_navigator` validates BOTH its default behavior trees on
activation - `navigate_to_pose` AND `navigate_through_poses` - not just
the one actual Nav2 goals use. The through-poses default tree contains a
`Wait` BT node, which needs the `wait` action server from
`behavior_server`. `behavior_plugins: ["spin", "backup"]` (no `wait`) left
that action server missing entirely, which failed BT validation and
aborted the whole lifecycle bringup - reproduced live: `"wait" action
server not available after waiting for 1.00s`, then
`lifecycle_manager_navigation` fails to activate `bt_navigator` and aborts
bringup for the whole stack. Fixed by adding `wait: {plugin:
"nav2_behaviors/Wait"}` to `behavior_plugins`. Matches the real
`webots_ros2_turtlebot` reference config, which also includes `wait` for
this exact reason.

**Both costmaps use an exact rectangular `footprint`, not `robot_radius`**
(for navigating tight 353mm doorways). Corner points are this robot's own
real chassis bounding box measured from `chassis.stl` (X: -0.1759..0.1165,
Y: -0.0824..0.0826 - see `base.urdf.xacro`'s mesh-vertex comment, rounded
to ±0.0825 for symmetry), not a generic vendor spec despite an unrelated
task description calling it a "RoboMaster X3" footprint - the numbers
matched our own robot exactly, so applied as given. `footprint` is parsed
as a STRING by `nav2_costmap_2d` (confirmed via `strings` on
`libnav2_costmap_2d_core.so` - `makeFootprintFromString` takes a
`std::string`), so the quoted `"[ [x,y], ... ]"` syntax is correct, not a
mistaken YAML sequence. Setting `footprint` supersedes `robot_radius`,
which was removed from both costmaps rather than left stale alongside it.

**Inflation tightened for the same reason: `cost_scaling_factor: 3.0 →
15.0`, `inflation_radius: 0.45 → 0.20`.** A 353mm doorway against this
robot's ~165mm Y-width leaves only ~94mm clearance per side - less than
the old 0.45m inflation radius, which would have saturated the entire
doorway with near-maximum cost from both jambs simultaneously. The
steeper `cost_scaling_factor` makes cost decay fast enough that the
doorway center should stay well under lethal, but this is a tuning
prediction, not something verified by an actual live traversal - watch
for the robot refusing to path through narrow gaps and reconsider these
two values first if it does.

**The `RotateToGoal` and `GoalAlign` DWB critics are differential-drive assumptions** (force
rotate-in-place toward the goal heading before approaching). Removed them because they fight 
this robot's ability to strafe directly at a goal. Only `Oscillation`, `BaseObstacle`, `PathDist`, 
`GoalDist`, and `PathAlign` are used. `PathAlign` specifically prevents crab-walking by keeping 
the robot body tangent to the planned path.

**Stop-Turn-Drive elimination.**
`smooth_path: true` is set in the `GridBased` global planner to generate fluid approach curves 
instead of sharp, non-holonomic pivot corners. DWB's `vy_samples` is raised to `20` and 
`min_vel_y`/`max_vel_y` are clamped to `±0.35` for dense, stable lateral motion sampling.

**Controller noise floor and goal jitter.**
`min_x_velocity_threshold`, `min_y_velocity_threshold`, and `min_theta_velocity_threshold` 
are raised from `0.001` to `0.01` to suppress actuator chatter. `SimpleGoalChecker` is tuned 
tight (`0.08`m / `0.15`rad) with `stateful: true` to enable clean hysteresis for goal settling 
without wiggling.

## EKF / IMU wiring (live-confirmed bugs, 2026-08-27)

**`Ros2IMU` publishes empty `frame_id` unless `<frameName>` is set.**
`sim_control.urdf.xacro`'s `<plugin type="webots_ros2_driver::Ros2IMU">` block
previously had no `<frameName>` tag. At runtime `header.frame_id` was `''`
(confirmed via `ros2 topic echo /imu/data --once --field header`).
`robot_localization`'s EKF cannot look up a sensor with no frame in the TF tree,
so all IMU measurements were silently dropped — the filter ran on odometry alone
and had no yaw correction, which is why rotation drift was visible.
Fixed: added `<frameName>imu_link</frameName>` to the plugin block.
This must match the URDF link name (`frame_id + "_link"` as expanded by the
`xacro:imu` macro in `amr.urdf.xacro` — currently `imu_link`).

**`linear_acceleration` from `Ros2IMU` is `NaN` in this Webots configuration.**
Live inspection (`ros2 topic echo /imu/data --once`) showed
`linear_acceleration.x: NaN`, `linear_acceleration.y: NaN`.
The Webots Accelerometer device is not delivering valid data to `Ros2IMU`
(likely because its sampling enable is tied to the Webots simulation step in
a way that isn't active during this launch configuration).
Fusing NaN into the EKF's state vector corrupts the entire covariance matrix and
produces erratic or locked `odom → base_link` output — the filter appeared to run
(no crash, no error) but its `x` and `y` estimates were frozen.
Fixed: disabled the `ax`/`ay` rows in `ekf.yaml`'s `imu0_config` (set both to
`false`). Only orientation (yaw) and angular velocity (vyaw) from the
InertialUnit and Gyro are fused — both were confirmed valid. If the accelerometer
is ever brought up properly, re-enable the last row of `imu0_config`.

**Odometry covariance was all zeros — EKF numerical degeneracy.**
`controllers.yaml`'s `twist_covariance_diagonal` and `pose_covariance_diagonal`
were `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`. `robot_localization` interprets a zero
diagonal as infinite certainty (zero uncertainty), which causes the Kalman gain
to degenerate — the filter either locks on the first measurement and never
updates, or produces tiny covariance values that cause numerical instability.
Fixed: set non-zero diagonals `[0.001, 0.001, 0.0, 0.0, 0.0, 0.005]` (x, y
position ~1mm uncertainty, yaw ~0.005 rad). Z/roll/pitch are left 0.0 because
`two_d_mode: true` ignores those axes anyway.

**RF2O Direct TF Publishing Architecture (2026-08-28).**
To eliminate IMU frame/covariance singularities and simplify dead-reckoning:
- EKF (`ekf_node`) disabled in `sim.launch.py`.
- `rf2o_laser_odometry` configured with `publish_tf: true` in `rf2o_params.yaml`, making RF2O the direct publisher of `odom -> base_link` TF transforms.
- Nav2 stack (`bt_navigator`, AMCL) configured to use `/rf2o/odom` and `odom -> base_link` transform tree directly.

**SmacPlanner2D & Webots Friction ContactProperties (2026-08-28).**
- `planner_server` configured with `nav2_smac_planner/SmacPlanner2D` and `smooth_path: false` to force straight-line global path planning and sharp grid-aligned turns.
- Webots `simulation.wbt` `WorldInfo` updated with `contactProperties` (material `rubber` on `default`, `coulombFriction: 1.5`) for physical traction.

**Instant Wiggle-Free SimpleGoalChecker (2026-08-28).**
- Configured `general_goal_checker` to `nav2_controller::SimpleGoalChecker` with `xy_goal_tolerance: 0.05` (5 cm spatial radius) and `yaw_goal_tolerance: 3.14` (180 deg). This terminates the path instantly upon entering the spatial radius without rotational micro-adjustments or wiggling.

## Lidar

**Two independent self-occlusion bugs, both had to be fixed, neither
alone was sufficient:**
1. Mount height: `laser_frame`'s xacro offset (`amr.urdf.xacro`) was
   originally `0.0825`, inherited from the source repo without
   re-measuring against *our* actual `chassis.stl` (whose real top is
   `z=0.1905`, measured directly from the mesh triangles). The sensor sat
   embedded inside the chassis body. Raised to `0.22`.
2. Even after that fix, all 360 rays still returned `.inf`. Root cause:
   `lidar.stl`'s own bounding box straddles zero on all 3 axes — the
   visualization dome mesh fully encloses its own local origin. Fixed by
   giving the `Lidar` node itself (not the mesh) a further
   `translation 0 0 0.03` inside `fix_proto.py`, so its ray-origin sits
   above the dome's own solid body.

**`type` must be `"rotating"`, not the schema default `"fixed"`.** Matches
the real RPLidar S2 hardware and Webots' own official `lidar.wbt` sample.
(An earlier theory that `"fixed"` silently clamps `fieldOfView` turned out
to be wrong — see the `--full-length` note below — but `"rotating"` is
still the technically correct choice for this hardware regardless.)

**`ros2 topic echo` truncates arrays to 128 elements by default.** Burned
us once: a "the Lidar only returns 129 samples out of 360" diagnosis was
actually just this truncation (`ros2 topic echo /scan --once` vs. the
correct `--full-length` or `--truncate-length N`). Always use
`--full-length` when inspecting `/scan` ranges.

**Lidar `frameName` must be set explicitly.** `webots_ros2_driver`'s
`Ros2SensorPlugin::init()` defaults a device's ROS `frame_id` to the
Webots device's own internal name (`"Lidar"`) unless `<frameName>` is
given in the `<device>` block's `<ros>` config
(`sim_control.urdf.xacro`). Since our TF frame is `laser_frame`, not
`Lidar`, omitting this means the LaserScan's `header.frame_id` can never
resolve a transform — `tf2`'s `MessageFilter` queues messages forever and
eventually crashes the subscriber (RViz or slam_toolbox) under sustained
backpressure ("Message Filter dropping message ... queue is full", then a
segfault). If you see that exact log pattern again on a *different*
topic/frame, check for this same missing-`frameName` class of bug first.

## RViz

**Never point a second display at `/scan` with a different message
type.** `/scan` is `LaserScan`. The point cloud is a genuinely separate
topic, `/scan/point_cloud` (`PointCloud2`) — confirmed in `Ros2Lidar.cpp`,
it publishes both on different topics, never the same one twice. Adding a
`PointCloud2` display pointed at `/scan` by mistake corrupts RViz's DDS
subscription state and crashes it (`invalid allocator`, SIGABRT). This
has happened more than once in this project — the saved
`viewport_config.rviz` is correct (one `LaserScan` display on `/scan`);
the crash comes from ad-hoc displays added live in the GUI.

**`Map` display: the default `map` color scheme crashes RViz (SIGKILL)**
on this machine's Mesa Intel driver — a GLSL shader link failure
(`indexed_8bit_image.frag`, "active samplers with a different type refer
to the same texture image unit"). Confirmed hardware-accelerated
(Mesa 23.2.1, Intel Xe, direct rendering — not a software-render
fallback issue). Workaround: set the `Map` display's `Color Scheme`
property to `costmap` instead of `map`. Unconfirmed whether this is
specific to this machine's Mesa version or a wider bug.

**`Fixed Frame` in `viewport_config.rviz` must be `base_link`**, not
`base_footprint` (see architecture.md) — it was originally
`base_footprint` and silently broke everything under `use_webots:=true`.

## Teleop

**`teleop_twist_keyboard` cannot run via `ros2 launch`.** It calls
`termios.tcgetattr(sys.stdin)` to enter raw keyboard mode, which requires
a real TTY. `ros2 launch`'s `Node` action pipes the child's stdin through
something that isn't a TTY (confirmed: `errno 25, ENOTTY`). The official
`turtlebot3_teleop` package ships no launch file at all for this exact
reason. Run it directly:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mecanum_drive_controller/reference_unstamped
```
`teleop.launch.py` still exists to document the correct remap, but will
always fail the same way if launched via `ros2 launch`.

**This robot is holonomic (mecanum)** — the Shift-held key variants in
`teleop_twist_keyboard` (`J`/`L`/`I`/`K` etc.) produce genuine sideways
strafing, which a differential-drive robot can't do. Worth knowing when
driving it for SLAM coverage.

## Process hygiene (testing discipline, not a code bug)

**`timeout N ros2 launch amr_simulation sim.launch.py` for automated
testing leaves orphaned processes.** `timeout` only signals the `ros2
launch` Python process; the real Webots app (`webots-bin`), its
`ros2_supervisor.py`, and the `webots_ros2_driver` process don't
reliably die with it, and pile up fighting over the same IPC port
(1234) across repeated test runs, causing spurious-looking failures
that have nothing to do with the code under test. If you must
script-test a launch, clean up explicitly afterward:
```
pkill -9 -f "webots-bin"; pkill -9 -f "/usr/local/webots/webots"
pkill -9 -f "ros2_supervisor.py"; pkill -9 -f "webots_ros2_driver"
pkill -9 -f "controller_manager/spawner"
rm -rf /tmp/webots/<username>
```
Prefer asking a human to run and observe a live launch over repeated
automated `timeout`-wrapped test cycles — it's disruptive to a real
GUI session and easy to leave in a broken shared state.

## Drivetrain: 2WD + caster, not mecanum (as of this transition)

The robot was fully re-platformed from a 4-wheel mecanum drive to a 2-wheel
differential drive + 1 passive caster, using the physical geometry from
github.com/ravithakur-projects/ROS-Three-Wheeled-Robot-Navigation-Project
("rmp_bot" - a `move_base`/ROS1 2WD+caster robot, despite the repo's "three-
wheeled" name referring to the caster as the third ground contact, not a
third driven wheel). Chassis shrank from 280x220x150mm to 200x170x110mm
(mass 4.6kg -> 7.21kg chassis-only, per rmp_bot's own base_link inertial).
Wheel radius (32.5mm) coincidentally matches the old mecanum wheels exactly;
wheel separation is now 170mm (was 169mm) and wheel width 25mm (was 30.4mm).

Everything downstream that assumed a holonomic/mecanum platform had to
change, not just the URDF:
- `controllers.yaml`: `mecanum_drive_controller` -> `diff_drive_controller`
  (ros-humble-diff-drive-controller 2.53.1). Unlike mecanum's
  ChainableController (no direct cmd_vel subscription, commands go to
  `.../reference_unstamped`), this build IS the classic topic-subscribing
  controller - it listens on `<name>/cmd_vel_unstamped` and publishes
  odometry on `<name>/odom`. Every place that published/remapped to
  `/mecanum_drive_controller/reference_unstamped` (twist_mux's cmd_vel_out
  remap in sim.launch.py, teleop.launch.py, visual_docker.py) had to move
  to `/diff_drive_controller/cmd_vel_unstamped`.
- `amcl` (both nav2_params.yaml's block AND the standalone amcl.yaml, which
  must be kept in sync per that file's own duplication note):
  `robot_model_type` changed from `nav2_amcl::OmniMotionModel` to
  `nav2_amcl::DifferentialMotionModel`. This isn't cosmetic - Omni models a
  particle motion prediction that includes lateral/strafing motion a 2WD
  robot physically cannot produce. Left on Omni, AMCL would silently predict
  motion the real robot can never make.
- `ros2_control.urdf.xacro`'s `webots_ros2_control::Ros2ControlSystem`
  plugin declaration stayed exactly where it already was, inside
  `<ros2_control><hardware><plugin>` - a prescriptive task for this
  transition asked to move it under the root `<robot><webots>` tag instead,
  which would have been wrong: that tag only accepts Webots-bridge plugins
  like `webots_ros2_control::Ros2Control` (a different class, already
  correctly in sim_control.urdf.xacro), not a `hardware_interface::
  SystemInterface` pluginlib registration. The two plugin types load
  through completely different mechanisms.
- Nav2's padded footprint shrank from ±0.17/±0.13m to ±0.13/±0.105m (same
  +3cm/+2cm margin convention as before, applied to the smaller chassis).
- The passive caster is modeled as a fixed-joint sphere (not a real swivel
  joint) with a dedicated near-zero-friction "caster_material"
  ContactProperties in simulation.wbt/fix_proto.py - it contributes no
  drive/steering DOF either way, so a low-friction fixed sphere is the
  standard simplification.

**Wheel mesh transform bug found right after this transition (real, not
cosmetic)**: the initial pass copied the old mecanum wheel's visual
`<origin xyz="0 0 0" rpy="1.5708 0 0"/>` verbatim onto the new rmp_bot wheel
meshes, which rendered both wheels floating in the wrong place/orientation
in both RViz and Webots (confirmed visually, not just in review). The two
mesh sources use different authoring conventions and are not
interchangeable: rmp_bot's `l_wheel_1.stl`/`r_wheel_1.stl` raw vertices are
already expressed directly in base_link's own absolute frame (a Fusion360
URDF-exporter quirk), which their own xacro compensates for by setting each
wheel's visual origin to the exact negation of its joint origin (e.g.
joint `-0.05 -0.085 0.0325` -> visual `0.05 0.085 -0.0325`) with **no**
rotation - the mesh's own bounding box already has its thin (25mm) axis on
Y, matching the joint axis. `wheel.urdf.xacro` now reproduces that
cancellation algebraically (`-wheel_xoff`, `-y_reflect*(wheel_separation/2)`,
`-wheel_radius`, `rpy 0 0 0`) instead of assuming every wheel mesh needs the
old 90-degree flip. Lesson: a mesh's required origin transform is a property
of *that specific mesh's* authoring convention, not something safe to copy
from a previous mesh even when the macro structure looks reusable.

**urdf2webots gotcha found during this transition**: passing `--robot-name`
together with `--output=*.proto` silently produces NO file at all (exit
code 0, no error) - `--robot-name` switches the tool into "generate a Robot
node string" mode instead of "generate a PROTO file" (confirmed by reading
importer.py's `isProto` branch). Omit `--robot-name` when writing a .proto;
the robot's internal name is derived from the output filename's basename
automatically.

## Map file location

Saved maps go to `~/ros/maps/` (e.g. `~/ros/maps/room_map.yaml` +
`.pgm`), via `ros2 run nav2_map_server map_saver_cli -f ~/ros/maps/room_map`.
This is a project-specific choice, not a Nav2 default — some
generic instructions elsewhere may say `~/map_files/`; that directory
does not exist in this project.

## TurtleBot3 correction round: own world, RF2O reinstated

Right after the first TurtleBot3 integration pass (see the "Migrated to
TurtleBot3 Burger" section above), a follow-up review caught two real
problems with it and asked for a fix. Both were verified independently
against source before applying, and one of the review's own proposed
fixes turned out to use a wrong parameter name.

**Problem 1 — wrong world was loading.** `sim.launch.py` originally
`IncludeLaunchDescription`d `webots_ros2_turtlebot`'s own `robot_launch.py`
verbatim, which defaults its `world` argument to
`turtlebot3_burger_example.wbt` and never gets that argument overridden.
Result: Webots was silently loading ROBOTIS's reference apartment, not
this project's own `simulation.wbt` (walls, AprilTags, docking corridor).
Fixed by forking `robot_launch.py`'s logic into `sim.launch.py` (see that
file's own docstring for the full rationale) and pointing `WebotsLauncher`
at `amr_simulation/webots/worlds/simulation.wbt` directly. Confirmed via
`inspect.getsource` on `launch.substitutions.path_join_substitution` that
`PathJoinSubstitution.perform()` joins with `str(Path(*components))` —
unlike `os.path.join`, `pathlib.Path` does **not** drop earlier components
when a later one is absolute, so passing an absolute world path as a
launch argument would NOT have worked without the fork anyway.
`simulation.wbt` itself needed the real `TurtleBot3Burger` PROTO added
(EXTERNPROTO'd from `cyberbotics/webots` at the `R2025a` tag, matching
this world's existing pin — verified both new proto URLs resolve at that
tag before using them) in place of the fully-dead `amr_robot` PROTO from
the earlier rmp_bot migration, with its `extensionSlot` (imu_link Solid,
GPS, InertialUnit, RobotisLds01) copied verbatim from ROBOTIS's own
reference world — the `RobotisLds01` slot specifically is what makes
`/scan` exist at all; `turtlebot_webots.urdf`'s device bindings expect
exactly that sensor set. `navigation.launch.py`'s `map` default was
reverted from `webots_ros2_turtlebot`'s bundled map back to this project's
own `~/ros/maps/room_map.yaml`, since we're spawning in our own world
again.

**Problem 2 — RF2O reinstated as the odom TF authority, not wheel
encoders.** The review argued rf2o's scan-matching odometry is more
reliable than wheel-encoder odometry for this setup and should be
authoritative again. Two of its proposed parameter values were checked
against real source and were wrong:
  - It said to set `publish_odom_tf: false` on `diffdrive_controller`.
    The real compiled parameter (confirmed via
    `diff_drive_controller_parameters.hpp`) is `enable_odom_tf`, not
    `publish_odom_tf`.
  - It said to set rf2o's `base_frame_id` to `"base_footprint"`. rf2o's
    own compiled default (confirmed via `CLaserOdometry2DNode.cpp`) is
    `"base_link"`, which is also what every other frame setting in
    `nav2_params.yaml`/`amcl.yaml` already uses — `base_footprint` only
    exists as a static zero-offset TF child of `base_link`, so pointing
    rf2o at it directly would have added an unnecessary extra TF hop for
    no benefit.
Since `diffdrive_controller`'s `ros2control.yml` is installed and has no
launch argument to override its path, disabling `enable_odom_tf` required
a project-owned copy: `amr_simulation/config/ros2control.yaml` (identical
to the installed file plus that one line), loaded by the forked
`sim.launch.py` instead of the installed one — the second reason that
fork was necessary. `rf2o_laser_odometry` was re-added to
`navigation.launch.py` with the exact same `TimerAction(period=3.0, ...)`
delay pattern used for it earlier in this project (a real, previously
diagnosed race: rf2o's one-shot `base_link -> laser_frame` TF lookup on
its first scan callback does not retry on failure), and
`nav2_params.yaml`'s three `odom_topic` settings were pointed back at
`/odom_rf2o`. Plain `/odom` (wheel encoders) is still published by
`diffdrive_controller` — only its TF broadcast is now disabled — so
nothing downstream actually reads it; it's there for comparison/debugging
only. This reinstatement is safer than it would have been earlier in this
project: the actual rf2o bug that originally caused stale-pose
republishing and recovery-loop oscillation
(`CLaserOdometry2DNode::process()` ignoring `odometryCalculation()`'s
return value) was fixed and committed before TurtleBot3 was ever
integrated, so that specific failure mode does not reappear here.

## Lidar Docker removed (visual/ArUco docking kept)

`amr_navigation/scripts/lidar_docker.py` — the reactive lidar-only
U-turn/docking controller — was removed completely at the user's request:
the script itself, its `CMakeLists.txt` install entry, its `Node` in
`sim.launch.py` (plus that file's header/twist_mux comments referencing
it), the now-unused `docking` channel in `twist_mux.yaml`
(`/cmd_vel_dock`, priority 50 — nothing else published to it), and the
mentions in `navigation.launch.py`'s comments, `architecture.md`, and the
Nav2 BT XML comment. One historical mention survives deliberately in this
file's "Twist Multiplexing" entry above — that's a record of the old
mecanum-era architecture (which also references
`/mecanum_drive_controller`, itself long gone), not live documentation,
so it wasn't rewritten, the same way git history isn't rewritten.

**`visual_docker.py`/`aruco_detector.py` (the separate ArUco-tag-based
docking system) were left untouched** — "Lidar Docker" and "visual
docking" are two independent systems that happened to share the word
"docking"; only the former was asked to go. Worth knowing if either is
touched again: `visual_docker.py` publishes straight to `/cmd_vel`,
bypassing `twist_mux` entirely — a pre-existing oddity, not something
this removal introduced or fixed.

## Narrow-passage navigation failure: wrong lever, then the real fix

The robot could not thread this layout's narrow passages (353mm being the
tightest — see the AS2/AS3/AS1 layout drawing, 3000x2500mm overall,
confirmed matching `maps/room_map.pgm`'s 62x51px @ 0.05m/px = 3.10m x
2.55m). Symptom in `maneuver_telemetry_node`'s log: `RECOVER_BACKUP` ->
`IN_PLACE_SPIN` -> `TRANSITIONING` looping indefinitely at the passage
mouth — recovery returning the robot to the exact geometry that just
failed, forever.

**First attempt (wrong): lowering `inflation_radius`/`cost_scaling_factor`
on both costmaps, repeatedly, across several tuning passes.** This could
never have worked, and the reasoning behind it was corrected once checked
against real behavior. `nav2_costmap_2d::InflationLayer` unconditionally
stamps every cell within the robot's *inscribed radius* (0.1m, from
`robot_radius` + `footprint_padding: 0.0`) as
`INSCRIBED_INFLATED_OBSTACLE = 253`, and `SmacPlanner2D` treats 253 as
untraversable — full stop, regardless of `inflation_radius` or
`cost_scaling_factor`, which only shape the cost gradient *beyond* that
253 core. Successive reductions (0.12 -> 0.10 -> 0.08 -> 0.06) were tuning
a parameter that does not govern the actual obstruction, and 0.06 was
actively worse: below the inscribed radius, it produced no useful
guidance gradient at all while still tripping nav2's own "configured
inflation radius is smaller than the computed inscribed radius" warning.
Lesson for next time: check what a parameter *actually gates* (via
`strings` on the real `.so`, as always in this file) before assuming a
plausible-sounding knob controls the symptom.

**Real root cause: costmap resolution, not inflation.** The traversable
band through a passage is `passage_width - 2*inscribed_radius`, fixed by
robot geometry and not reducible by inflation tuning at all. For the
353mm passage that's 153mm — at the map's native 0.05m/px, that's ~3.1
grid cells, and grid misalignment routinely rounds that down to 2 or 0
cells of connectivity, which is exactly what was failing. Compounding
this: `nav2_costmap_2d::StaticLayer` unconditionally forces the *global*
costmap to adopt the loaded map's own resolution ("StaticLayer: Resizing
costmap to %d X %d at %f m/pix", confirmed via `strings` on
`liblayers.so`) — so `global_costmap.resolution` in `nav2_params.yaml` was
silently a no-op the whole time; the only way to change it is to change
the map itself.

Fix: generated `maps/room_map_hires.pgm`/`.yaml` — a 2x nearest-neighbour
upsample of `room_map.pgm` (PIL `Image.NEAREST`, no interpolation, so
occupancy values are preserved exactly; pixel histogram is exactly 4x the
original's, same 3.10m x 2.55m extents, same origin) at 0.025m/px instead
of 0.05m/px. `navigation.launch.py`'s `map` launch argument default now
points here instead of `room_map.yaml` (which is untouched — point the
argument back to revert). `local_costmap.resolution` was set to 0.025
directly in `nav2_params.yaml` (no static layer involved there, so this
one *is* authoritative). At the new resolution the same 153mm band is
~6.1 cells — an actual corridor to route through instead of a coin-flip.
`planner_server.GridBased.downsample_costmap` was explicitly pinned
`false` (a real `SmacPlanner2D` parameter, confirmed via `strings` on
`libnav2_smac_planner_2d.so`) so the planner can't quietly throw away that
resolution gain on its own. `cost_travel_multiplier` was also lowered
2.0 -> 1.0: at the old value the planner would rather detour around a
passage than accept the cost of threading it, which is backwards when
threading it is the only way through.

**Controller swap: DWB -> MPPI, since DWB's failure mode here is
structural, not a tuning problem.** `dwb_core::DWBLocalPlanner` scores a
coarse *discrete* grid of `(vx, vtheta)` sample pairs (20 x 35 in this
project's tuning) and rejects any whose swept cells are too costly. In an
11cm-wide free corridor, the lateral spread between adjacent DWB samples
at any useful speed is often wider than the corridor itself — whole
regions of the sample grid land in the inflated band simultaneously, DWB
finds zero admissible rollouts, and `controller_server` reports failure,
which is what was driving the backup/spin recovery loop. Switched the
active `FollowPath` to `nav2_mppi_controller::MPPIController` (confirmed
installed: `libmppi_controller.so`/`libmppi_critics.so`; every parameter
name and critic below was read directly off those two libraries, and
`"DiffDrive"` confirmed as a literal valid `motion_model` string via the
library's own validation message). MPPI samples a large *stochastic*
batch (2000 rollouts) around the previous optimal control sequence and
returns a cost-weighted average rather than picking one discrete winner,
so it degrades gracefully in a corridor where most samples are poor
instead of failing outright when all of them are. `dwb_core::
DWBLocalPlanner` is preserved, unchanged, as `FollowPathDWB` (not in
`controller_plugins`, so inactive) — swap the two block names back to
revert. `nav2_regulated_pure_pursuit_controller` remains preserved as
`FollowPathRPP` from an earlier pass, also inactive, for the same reason
(see this file's earlier Nav2-stack entries for why RPP itself was
originally swapped out).

MPPI's critics carry forward two earlier, still-standing requests from
this project: `vx_max`/`wz_max` (0.35 / 0.6) keep "straight-line can move
at normal speed, rotation should be slow" from the DWB-era tuning, and
`TwirlingCritic` carries forward "prefer straight-line commands over
curvier alternatives" the same way it was added (then, alongside DWB,
temporarily reverted mid-session) before landing here on MPPI instead.
`CostCritic` (not `ObstaclesCritic`) was chosen deliberately: `Obstacles
Critic` reconstructs distance-to-obstacle by inverting the inflation
formula, which requires its own `inflation_radius`/`cost_scaling_factor`
copies to exactly mirror the costmap's — a fragile coupling in a file that
retunes inflation often. `CostCritic` scores the costmap value directly,
so it stays correct through future inflation changes without needing a
second copy of those numbers.

## Goal-reaching tolerance loosened (was hunting forever near the goal)

`nav2_controller::SimpleGoalChecker` (confirmed via `strings` on
`libsimple_goal_checker.so` — only `.xy_goal_tolerance`/
`.yaw_goal_tolerance`/`.stateful` exist, no velocity check at all) only
reports the goal reached once position AND yaw are *simultaneously* within
tolerance. At the original 8cm/~8.6°, the robot would reach the position
window easily but then spend a long time making small in-place
corrections chasing the last few degrees of exact final heading —
visibly "stuck" spinning right next to the goal instead of ever
completing. Loosened both `general_goal_checker` and `FollowPath`'s own
copy (the two must match, per this file's — and the params file's own —
existing comment) to `xy_goal_tolerance: 0.15` / `yaw_goal_tolerance:
0.35` (~20°), so "close enough" actually registers as SUCCEEDED.

## Angular speed halved, linear speed untouched

Per a direct request ("really really slow" turning, straight-line motion
can stay at normal speed): every rotational-speed ceiling in the stack
was cut roughly in half, while every linear (x) limit was left exactly as
it was. Touches three independent places that all needed to move
together for a consistent feel: the active local controller's own angular
ceiling and accel/decel (`wz_max`/equivalent), `velocity_smoother`'s theta
entries in its `max_velocity`/`min_velocity`/`max_accel`/`max_decel`
arrays (the x entries in those same arrays are untouched), and
`behavior_server.spin`'s `max_rotational_vel`/`rotational_acc_lim` (the
Spin recovery behavior's own separate rotational speed — not read from
the controller's params at all, easy to forget as a fourth place). If
rotation speed is ever retuned again, check all three/four spots, not
just the local controller's block — this project has already been bitten
once by `velocity_smoother`'s ceiling silently overriding a tighter
controller-side limit.

## Global costmap ignoring live LIDAR clearing near the robot's own pose

A real, live-diagnosed failure: the robot sat completely motionless while
`planner_server` repeated `"Starting point in lethal space! Cannot create
feasible plan"` indefinitely, yet in the same instant `Spin` completed a
full rotation and `BackUp` began moving - both without any collision
refusal from their own live local-costmap check. That split is the
diagnostic signature: `local_costmap` has no `static_layer` (pure live
LIDAR, see its `plugins` list), but `global_costmap` layers `static_layer`
(the pre-built map) under `obstacle_layer` (live LIDAR) and combines them
via `combination_method: 1` (Max, the default) - so a cell `static_layer`
marked as wall stays lethal forever even when the live scan's clearing
ray-traces straight through it and proves it's clear right now
(`max(clear, wall)` is still `wall`). A centimeter or two of map-to-reality
drift is enough to pin a cell like that in a passage this tight, and
neither `ClearEntireCostmap` nor any inflation setting fixes it -
`static_layer`'s own content isn't touched by a costmap clear, and the
cell in question was never a soft decay value to begin with.

**Fix: `global_costmap.obstacle_layer.combination_method: 0`
(Overwrite)** - confirmed real parameter via `strings` on `liblayers.so`.
Now the live scan's own clearing can overwrite a stale static-map wall
cell instead of only ever being able to raise its cost. Cells the current
scan doesn't observe this cycle are untouched either way, so a wall
genuinely outside the LIDAR's current view still comes from
`static_layer` unaffected. This is the standard, documented Nav2 lever
for "trust the live sensor over the pre-built map" - unlike
`inflation_radius`/`cost_scaling_factor`, which cannot help here at all,
since the blocking cell was the static map's own hard entry, never a soft
gradient.

Also corrected in the same pass: a stale comment on `behavior_server.spin`
described the OLD rectangular mecanum chassis's swept-corner risk. This
robot's costmaps declare it with a plain `robot_radius` (circular, no
footprint polygon anywhere in this file) - a circle rotated about its own
center sweeps exactly the disk it already occupies at rest, so in-place
rotation adds zero additional collision exposure for this robot,
unlike the old chassis. And `BackUp`'s BT-side distance/speed
(`backup_dist`/`backup_speed` in the navigate-to-pose BT XML) was
right-sized from an oversized 0.30m/0.05m/s down to Nav2's own stock
default, 0.15m/0.025m/s - the larger distance routinely exceeded real
available clearance in this layout's tight corridors, so `BackUp` would
immediately re-trigger its own collision check (`Collision Ahead -
Exiting DriveOnHeading`, confirmed real live-firing log line via `strings`
on `libnav2_back_up_behavior.so`) and exit having moved almost nothing -
which read externally as "recovery not coming back from it," when the
recovery behaviors were in fact correctly refusing to make a bad position
worse.

## Ultrasonic proximity sensors added (LIDAR minimum-range blind spot)

The 2D LIDAR (`RobotisLds01`/`LDS-01`) has a minimum reliable detection
range that leaves it blind to obstacles very close to the robot's own
body - exactly the regime this layout's narrowest passages (down to
153-160mm of real free centerline, see the "Narrow-passage navigation
failure" entry above) push the robot into. Four Webots `DistanceSensor`
nodes (`sonar_front`/`sonar_left`/`sonar_back`/`sonar_right`) were added
to `simulation.wbt`'s `TurtleBot3Burger.extensionSlot`, mounted flush with
the robot's 0.1m hull radius at the front/left/back/right compass points
(the chassis is circular, not rectangular, so this is the closest real
equivalent to "four corners" - real blind spots remain at the diagonals,
an honest coverage gap, not something four sensors at 90 degrees apart
closes).

A user-supplied integration plan for this (from an external source, not
verified against this project's actual installed packages) was checked
against real source before implementing, and two of its claims were
wrong:
- It specified `radiation_type: ULTRASOUND` for the published
  `sensor_msgs/Range` messages. Confirmed via reading
  `webots_ros2_driver`'s own `Ros2DistanceSensor.cpp` directly: it
  hardcodes `sensor_msgs::msg::Range::INFRARED` regardless of the Webots
  node's own `type` field. Cosmetic only -
  `nav2_costmap_2d::RangeSensorLayer` never reads `radiation_type` - but a
  real inaccuracy in what actually gets published, worth knowing if
  anything downstream ever branches on it.
- It listed `phi: 0.26` as a `RangeSensorLayer` YAML parameter. Confirmed
  via `strings` on `liblayers.so` that no such top-level parameter exists
  - the layer's internal `phi_v_` comes from each incoming
  `sensor_msgs/Range` message's own `field_of_view` field at runtime, not
  a static config value. Left out of `nav2_params.yaml`'s `range_layer`
  block entirely rather than setting a parameter that does nothing.
- It also assumed this project's `amr_description/urdf` xacro ->
  `urdf2webots` -> `fix_proto.py` -> `amr_robot.proto` pipeline
  (`proto-regeneration.md`) was the way to get new sensors into the
  simulation. That pipeline targets `amr_robot.proto`, which
  `simulation.wbt` has not referenced at all since the TurtleBot3 pivot -
  Webots now spawns `TurtleBot3Burger`, cyberbotics' own external PROTO
  (see the "TurtleBot3 correction round" entry above). Running that
  pipeline for this would have regenerated a file Webots never loads.
  Confirmed instead (`WebotsNode.cpp`'s device-type switch) that
  `webots_ros2_driver` auto-instantiates a `Ros2DistanceSensor` plugin for
  *every* `WB_NODE_DISTANCE_SENSOR` device found on the robot with zero
  `<device>` XML entry required at all - adding the sensors directly to
  `simulation.wbt`'s `extensionSlot` (the same place `imu_link`/`GPS`/
  `RobotisLds01` already live) is the actual, current mechanism, no xacro
  or PROTO regeneration involved.

Real min/max range and field-of-view come from the Webots node's own
`lookupTable`/`aperture` fields (set in `simulation.wbt`), not from any
ROS-side parameter - `sensor_msgs/Range.min_range`/`max_range` are
*derived* by `Ros2DistanceSensor.cpp` from the lookup table's endpoints
adjusted by their noise margins, not independently settable.

Topic names (`/ultrasonic/front` etc.) needed a fork of
`webots_ros2_turtlebot`'s installed `resource/turtlebot_webots.urdf` into
`amr_simulation/resource/turtlebot_webots.urdf` (same "can't edit an
installed third-party file in place" situation `ros2control.yaml` hit
earlier in this project) purely to override each sensor's default
`"~/<device_name>"` topic (which resolves relative to the driver node's
own name) with an explicit, predictable absolute path -
`sim.launch.py`'s `robot_description_path` now points at this fork
instead of `webots_ros2_turtlebot`'s copy.

`nav2_costmap_2d::RangeSensorLayer` was added to `local_costmap.plugins`
only (not `global_costmap` - a sensor with a 0.5m max range has nothing
useful to contribute to long-horizon global planning), ordered before
`inflation_layer` so inflation is computed over the combined obstacle +
range-sensor picture. This closes the LIDAR blind spot at the costmap
level, which is what actually produces the "slow down near walls"
behavior requested: MPPI's existing `CostCritic`/`ConstraintCritic`
already penalize proximity to costed cells, so newly-marked close-range
obstacles feed into the same scoring that already exists - no separate,
bespoke speed-governor logic was needed or added.

## Ultrasonic sensor range corrected + nav2_collision_monitor added

Two follow-ups after the ultrasonic sensors above went in:

**Range was wrong by an order of magnitude.** The initial `lookupTable`
used a 0.02-0.5m range (matching the earlier general "close-range" ask),
but 0.5m is deep into LIDAR's own good working range and, in a
350-460mm-wide passage, reaches clean across to the far wall - visually
indistinguishable from a second LIDAR, not a hull-hugging proximity
sensor. Live-diagnosed from a screenshot showing the sensor's debug rays
stretching far past the robot's body. Shrunk to 0.01-0.08m (real few-cm
range) on all four `sonar_*` nodes in `simulation.wbt` - deliberately
small enough to stay irrelevant everywhere except genuinely close to a
wall, so it can never compete with or override LIDAR-based planning in
open space. The 0.1m-from-centre mounting height/position was NOT a bug -
checked against a real cached `TurtleBot3Burger` PROTO
(`.../webots_ros2_examples/protos/TurtleBot3Burger_enu.proto` found
locally) and its own body geometry centres almost exactly on the z=0.08
height already used here.

**`nav2_collision_monitor` added as a dedicated safety layer** - a
different mechanism from `local_costmap.range_layer` above, and a real,
separate request: "maintain constant clearance, and if it gets
dangerously close, react immediately" describes an independent reflex
that can't be out-voted by path-following priorities, not another input
for MPPI's critics to weigh. Confirmed installed
(`nav2_collision_monitor/collision_monitor_node.hpp`) with a purpose-built
`Range` source type for this exact sensor class (its own header docstring
reads "Implementation for IR/ultrasound range sensor source"). Wired in
as the actual last stage before the robot: `sim.launch.py`'s `twist_mux`
now outputs to `/cmd_vel_premonitor` instead of `/cmd_vel` directly, and
`collision_monitor` (launched in `navigation.launch.py`, added to
`lifecycle_manager_navigation`'s `node_names`) reads that and republishes
the real final `/cmd_vel` - deliberately spliced in AFTER twist_mux, not
before, so it sees and can override teleop commands too, not just Nav2's.

Two `circle`-type zones (matching this robot's real circular
`robot_radius`, not an arbitrary rectangle): `CollisionStop` at radius
0.12 (robot_radius 0.1 + 2cm - "if it moves another few cm, collision has
to occur," as directly requested) hard-stops via `action_type: "stop"`;
`CollisionSlow` at radius 0.16 (robot_radius + 6cm) reduces speed to 30%
via `action_type: "slowdown"`. Both radii are measured from `base_link`
(the zone is centred on the robot, not the hull surface), and both sit
comfortably inside the sensors' own 0.08m max range so an approach can
actually be detected before either boundary is crossed.

## Robot stalled completely: local_costmap resolution was the real cost driver

After the ultrasonic sensors/`collision_monitor` went in, the robot
stopped making any real progress: `controller_server` spammed `"Control
loop missed its desired rate of 20.0000Hz"` on nearly every cycle, its
own `local_costmap/clear_entirely_local_costmap` service started timing
out (the node starving its own callbacks, not just the control loop
timer), and `"Failed to make progress"` aborted paths repeatedly.

**First fix, insufficient alone: `slowdown_ratio: 0.3` on
`CollisionSlow` was compounding with MPPI's own caution.** In this
layout's passages, even perfect centering only clears 7.65cm per side -
this slowdown zone (triggers under 6cm) is active for nearly the entire
time the robot is inside any of them, by design. But MPPI's own
`CostCritic`/`ConstraintCritic` were ALREADY slowing it down for the same
proximity, and multiplying an already-small commanded velocity by 0.3
routinely landed below `controller_server.min_x_velocity_threshold`
(0.01), which zeroes it outright - the robot would sit motionless inside
a zone it couldn't escape, then fail `progress_checker`'s
movement-in-time check. Raised to 0.6 (a real but much milder cut) - this
alone did not fix the "missed its desired rate" spam, which turned out to
be a separate, bigger problem.

**Real cause: `local_costmap.resolution: 0.025` was never actually
needed and was the biggest new compute cost.** That value was set to fix
`SmacPlanner2D`'s GLOBAL grid-search feasibility through 353mm passages
(see the "Narrow-passage navigation failure" entry above) - correct
reasoning there, but it was mistakenly carried over to the LOCAL costmap
under the same logic, which never applied: MPPI doesn't grid-search, it
scores continuous-space trajectories against costmap values, and 5cm
cells are already several times finer than this layout's ~7.65cm
worst-case clearance margin. What 0.025 actually cost: a 3x3m rolling
window at that resolution is 14400 cells vs 3600 at 0.05 - a real 4x
increase in what `inflation_layer`/`range_layer` recompute on every
costmap update, all inside `controller_server`'s own single-threaded
executor (confirmed no configurable multi-threaded executor exists for
this Humble build - checked `strings` on the installed `controller_server`
binary and `libcontroller_server_core.so`, neither declares an
`executor_type`/`num_threads` parameter - this is not a flag being
missed, it genuinely isn't available here). That executor is the SAME
thread MPPI's rollouts run on, so the 4x-larger costmap recompute was
starving the control loop timer AND the node's own service callbacks.
Reverted to 0.05. MPPI's `batch_size` (2000 -> 1000 -> 500) and
`time_steps` (56 -> 36) were also cut alongside this, since halving
batch_size alone (2000 -> 1000, done first, before the resolution cause
was found) did not clear the warnings by itself.

Lesson for next time: when a resolution/fidelity increase is justified
for one specific reason (here, a discrete grid-search planner needing
enough connected cells), check whether that reasoning actually extends to
every place the same-sounding parameter exists (here, local vs global
costmap) before copying the value across - it does not automatically
transfer to a different consumer with a different algorithm.

## Spin's rotation speed was silently ignored: wrong YAML nesting

Live-diagnosed via `ros2 param get /behavior_server spin.max_rotational_vel`
returning `"Parameter not set"`, then `ros2 param list /behavior_server`
showing `max_rotational_vel`, `min_rotational_vel`, `rotational_acc_lim`,
and `simulate_ahead_time` all existing ONLY as top-level
`behavior_server` parameters - not nested under `spin:`/`backup:` as this
file previously claimed ("every per-behavior TimedBehavior parameter
follows this nesting convention, same as progress_checker/
general_goal_checker" - that claim was wrong, and had never been checked
against the actually-running node's real parameter list, only inferred
from `strings` on the plugin library, which confirms a parameter *name*
exists but says nothing about what prefix it's declared under). This
installed `nav2_behaviors` version declares these four without a
per-plugin prefix, shared across whichever behaviors use them (Spin's own
rotation speed; `simulate_ahead_time` shared by Spin/BackUp/
DriveOnHeading's common `CostmapTopicCollisionChecker` ahead-check -
confirmed all three link against that same class). The nested copies
under `spin:`/`backup:` had been silently ignored for most of this
project's history - Spin ran on its compiled default (observed live:
`w=1.00 rad/s`) regardless of what the file said, right up until this
was caught.

Moved all four to the real top level of `behavior_server.ros__parameters`.
General lesson, worth repeating given this is the second time it's bitten
this exact project: `strings` on a compiled plugin proves a parameter
*name* is real, but not where in the YAML hierarchy it's actually read
from - `ros2 param list`/`ros2 param get` against the live, running node
is the only way to confirm the second part, and should be checked
whenever a "why isn't this taking effect" symptom shows up, not assumed
from naming convention alone.

## Control loop's real rate ceiling was range_layer, not the LIDAR

`controller_frequency: 20.0` was live-measured (off the "Control loop
missed its desired rate of 20.0000Hz" warning's own timestamps) to be
only actually achieving 4.33Hz - the minimum gap between consecutive
warnings was 0.231s. The first hypothesis was that this matched the
Webots LDS-01's own measured 4.45Hz `/scan` rate (see rf2o_params.yaml's
`freq: 4.0` reasoning) and that the control loop simply couldn't outrun
its own pose source - `controller_frequency` was set to 5.0 and MPPI's
`model_dt`/`time_steps`/`batch_size` rebuilt around that on that theory.

That theory was wrong, caught by testing rather than assumed correct:
live-set `controller_frequency` to 10.0 via `ros2 param set` and measured
`/cmd_vel_raw` at exactly 10.001Hz with **zero** missed-rate warnings over
a 40-second window. The real cause was `local_costmap.range_layer` -
four sonar topic subscriptions plus per-scan cone-marking, recomputed
every cycle on `controller_server`'s own single-threaded executor (the
same one MPPI's rollouts run on - see the "Robot stalled completely"
entry above for why no multi-threaded executor exists here to absorb
that cost). Removing `range_layer` from `local_costmap.plugins` freed
the loop entirely; `controller_frequency: 10.0` is the value actually
running now (twice the ~4.5Hz pose rate - fast enough that a lost cycle
still gets a fresh pose reading soon after, not so fast that it just
re-consumes the same rf2o pose several times for nothing). `model_dt`
was corrected to `0.1` (== `1/controller_frequency` - this MUST always
hold, see below) and `time_steps` to `28` (same ~2.8s/~1m horizon as
before, resampled at the corrected step size).

The sonar readings didn't disappear - they still feed
`nav2_collision_monitor` directly (see below), which was always the
correct consumer for a sub-robot-radius-range sensor; this only removes
them from the costmap MARKING path, which - separately - turned out to
be actively harmful (next entry).

Lesson: measure before committing to a theory that "explains" a number,
even a plausible one backed by a real, previously-confirmed fact (the
LIDAR rate). `ros2 param set` on a live node costs nothing to try before
rewriting a config file around an assumption.

## MPPI's model_dt must equal 1/controller_frequency, always

`FollowPath.model_dt` is the time-step MPPI integrates each rollout with
internally - it is not a free tuning knob, it must equal the actual
control period the optimizer's first command will be held for. This was
left at a stale `0.05` while `controller_frequency` moved 20 -> 5 -> 10
(effective real rate 4.33 -> 5 -> 10Hz across this debugging session),
meaning for most of that time MPPI believed its commands would be
refreshed far more often than they actually were, so it consistently
either under- or over-shot each planning window. Whenever
`controller_frequency` changes, `model_dt` changes with it in the same
edit - both are set from the same `1/controller_frequency` value and
`time_steps` re-derived to keep the total horizon (`time_steps *
model_dt`, currently `28 * 0.1 = 2.8`s) at roughly the same real-world
lookahead distance the passages in this layout call for, rather than
left at whatever step count happened to be there before.

## Ultrasonic sensors were mounted in the wrong frame: extensionSlot != base_link

`simulation.wbt`'s `TurtleBot3Burger.extensionSlot` is **not** an alias
for the robot's own origin - it is offset to `(-0.03, 0, 0.153)` in
`base_link`, which is exactly the LDS-01 lidar's real mount point (that's
why `RobotisLds01 {}` needs no `translation` field at all: it already
sits at the slot's origin). The four sonar `DistanceSensor` nodes were
first written with `translation`s intended as base_link-relative (e.g.
`0.1 0 0.08` for "10cm out, 8cm up" on the front sensor) - Webots applied
them slot-relative instead, live-confirmed via
`ros2 run tf2_ros tf2_echo base_link sonar_front` reporting the frame at
`(0.070, 0, 0.233)`, not `(0.10, 0, 0.08)`: 3cm too far back and 15cm too
high. That 0.233m height is the "distance array way above the TurtleBot
marker" visual reported live, and the 0.070m radial placement put the
front sensor's own MAX-RANGE return point at only 0.1498m from
base_link - permanently inside `collision_monitor`'s 0.16m `CollisionSlow`
ring, which is why the robot drove every goal in a constant 60% slowdown
that toggled with ordinary sensor noise (58 toggles measured in one
60-second window off `/rosout`).

Fixed by back-solving each translation as `(desired base_link pose -
slot origin)`, putting all four sensors at exactly 0.10m from centre,
z=0.08 (mid-body: above the 0.066m wheel tops, below the 0.153m lidar
deck) as originally intended. A detailed warning comment is left directly
above these translations in `simulation.wbt` itself.

Lesson: never assume a PROTO's `extensionSlot` (or any node's declared
child-attachment point) coincides with that node's own reference frame -
verify any newly-added device's real TF against `base_link` with
`tf2_echo` before trusting a translation value, especially when the
sibling nodes already in that slot (like `RobotisLds01 {}` here) offer a
visible clue (zero translation on a sensor that is definitely not at the
robot's centre) that the slot has its own non-trivial offset.

## MPPI sampling noise must be sized as a fraction of THIS robot's velocity limits, not left at Nav2's stock defaults

`vx_std: 0.2` / `wz_std: 0.4` are Nav2's own stock example values, sized
for the stock example robot's `vx_max: 0.5` / `wz_max: 1.9`. This project
runs `vx_max: 0.35` (later raised to `0.45`) and a deliberately halved
`wz_max: 0.6` - against those limits the stock noise values were 57% of
the linear range and 67% of the angular range, meaning the sampled
control distribution was close to uniform across the entire commandable
envelope. Live-observed consequence: with the robot sitting still and a
goal ahead, `w` oscillated `+0.29 -> -0.33 -> +0.28` in a clean limit
cycle and peak `vx` never exceeded `0.06` (out of `0.35`) over a full
60-second goal attempt, with 35 `IN_PLACE_SPIN` telemetry states logged
in that one window - every batch of rollouts contained roughly as many
hard-left as hard-right samples, so the cost-weighted average cancelled
to near zero instead of committing to either.

Reduced to `vx_std: 0.10` (~22% of `vx_max: 0.45`) and `wz_std: 0.15`
(25% of `wz_max: 0.6`), with `temperature` lowered `0.3 -> 0.2` for a
sharper (less averaged, more decisive) softmax pick over rollout costs -
this matters specifically in this layout's passages, where a left-detour
and a right-detour trajectory can score near-identically and averaging
them drives straight at the wall between them. Re-run under the same
conditions: 2 `IN_PLACE_SPIN` states (down from 35) and `STRAIGHT_FORWARD`
states appearing for the first time in that goal attempt.

Lesson: any critic/sampling-noise value copied from Nav2's own example
config is scaled for that example's velocity limits, not this robot's -
check it as a *fraction* of this project's actual `vx_max`/`wz_max`
(roughly 20-25% is a reasonable working range) rather than trusting the
literal number.

## PathAlignCritic has a silent kill-switch: max_path_occupancy_ratio

`PathAlignCritic` is not just a weighted cost term - past a threshold it
stops scoring entirely. It counts what fraction of the global path's
sampled points currently sit in high-cost/unknown local-costmap cells,
and once that fraction exceeds `max_path_occupancy_ratio`, the critic
returns **without scoring anything at all**: MPPI is no longer pulled
toward the global plan by this critic in that cycle and free-runs on
whatever its other critics alone produce. This project had it set to
`0.05` - stricter than Nav2's own `0.07` default - and that threshold
was tripping inside exactly the passages where staying aligned to the
global plan matters most: the plan through a 0.324m passage runs about
0.162m from both walls, and with `inflation_radius` close to
`robot_radius` only a few cm of ordinary AMCL/odom drift is enough to
push a run of path points into high-cost cells and cross the ratio. This
was directly responsible for the "local controller isn't following the
global path in the passages" behaviour reported live - the critic
wasn't mis-weighted, it had switched itself off.

Raised to `0.5` (permissive - appropriate for a robot whose whole job is
threading gaps barely wider than itself) and `PathFollowCritic.cost_weight`
raised `5.0 -> 8.0` alongside it, so the two path-following critics pull
together (align = stay on the plan laterally, follow = advance down it)
rather than PathAlign alone fighting the obstacle-avoidance critics once
it's allowed to actually run inside a tight passage.

Lesson: a critic's `enabled`/`cost_weight` pair is not always the whole
story - some critics (confirmed here via `nav2_mppi_controller`'s own
source) have their own independent activation thresholds that silently
disable them under specific costmap conditions. Worth checking a
critic's actual source, not just its declared parameters, when its
effect seems to vanish under specific conditions (like "only inside
narrow passages") rather than assuming it's a weighting problem.

## inflation_radius == robot_radius is not a valid tuning value, at any inflation_radius

Setting `inflation_radius` exactly equal to `robot_radius` (both `0.10`
at one point, on direct request to remove the soft inflation margin
entirely) is not merely aggressive tuning - it is a logical dead end,
independent of what the exact number is. `InflationLayer`'s only cost
transition (from `0` to `253`, `INSCRIBED_INFLATED_OBSTACLE`) then sits
exactly on the robot's own hull radius. A robot resting flush against a
wall - which is exactly what "drive straight into the end wall and stop"
produces, and a legitimate outcome of ordinary approach behaviour - then
has its own occupied cell sitting precisely on that transition boundary,
and a few millimetres of completely ordinary AMCL/rf2o pose noise between
update cycles is enough to flip that cell between reading fully clear and
fully lethal on every single costmap update. That flicker is the
"jittery/glitchy inflation" visually reported live in RViz screenshots -
it is not LIDAR sensor noise to be filtered, it is this exact geometric
coincidence.

It also defeats every recovery path at once, which is what made the
robot un-recoverable after getting wedged rather than merely
slow to escape: `Spin` and `BackUp` are both collision-checked against
the local costmap (confirmed live via "Collision Ahead - Exiting
Spin"/"...Exiting BackUp" log lines firing during normal operation
elsewhere in this project). With zero margin, the robot's own current
pose reads as already colliding, so both behaviours reject themselves as
unsafe on their very first attempt - and MPPI, evaluating trajectories
from that same already-critical starting cost, sees forward, reverse,
and rotate-in-place candidates all scoring near-equally badly every
cycle. The result observed live matched this exactly: 20+ seconds of
`IN_PLACE_SPIN <-> TRANSITIONING <-> IDLE_STATIONARY` cycling with `vx`
pinned under `0.01`, ending in `Failed to make progress` with no
recovery behaviour ever visibly succeeding, repeating identically on a
freshly-issued goal rather than clearing.

Settled on `inflation_radius: 0.12` (2cm of real margin - not the `0.25`
tried and explicitly rejected as visually too heavy) with
`cost_scaling_factor` raised `8.0 -> 14.0` so most of that thin 2cm band
decays to a low, faint cost rather than reading as a strong bright ring -
keeping the visible halo close to what `0.10` looked like while removing
the exact-zero-margin flicker. Verified this changes nothing about which
passages are plannable: connectivity of the traversable region (cells
with clearance > `robot_radius`, which is what `SmacPlanner2D` actually
refuses) was checked via a distance-transform on `room_map_hires.pgm`
and confirmed identical (one connected region, same cell count) at every
`inflation_radius` value tried in this session (`0.10`, `0.25`, `0.12`) -
`inflation_radius` only ever shapes a soft cost preference beyond the
hard inscribed band, never what gets refused outright.

Lesson: "reduce inflation to the minimum" and "set inflation_radius equal
to robot_radius" are not the same request - the latter removes a
necessary safety margin around the robot's own hull, not just around
obstacles, and produces exactly the kind of intermittent, hard-to-explain
"but it was working a second ago" failure this entry documents. A small
non-zero margin (a few cm, not the ~15cm previously rejected) combined
with a steep `cost_scaling_factor` satisfies both "looks thin in RViz"
and "the robot's own position never sits on a knife-edge" at the same
time - they are not actually in tension once separated like this.

## Linear speed raised, reverse capped independently

`vx_max: 0.35 -> 0.45` on direct request; `wz_max` (angular) was
deliberately left untouched at its earlier-halved `0.6` value - the
request was specifically for linear speed. `vx_min` was set to `-0.25`
rather than mirrored to `-0.45`: live telemetry during the MPPI-noise
debugging above showed long stretches of `RECOVER_BACKUP` and negative
`vx` as MPPI dithered backwards without ever being asked to by an actual
recovery behaviour; reverse is needed for genuine recovery (`BackUp`'s
own `backup_speed: 0.025` is unrelated and far smaller) but has no
reason to be as fast as forward travel, and capping it removes a
meaningful slice of unproductive backward rollouts from every MPPI
batch. `velocity_smoother`'s `max_velocity`/`min_velocity`/`max_accel`/
`max_decel` were all updated to match (`[0.45, 0.0, 0.6]` /
`[-0.25, 0.0, -0.6]` / `[0.7, 0.0, 0.6]` / `[-0.7, 0.0, -0.6]`) - the
smoother's envelope must cover MPPI's own limits on both bounds, or it
silently clips whatever MPPI asks for.

## Open-aisle wall-hugging + un-recoverable narrow-passage deadlock: a computed fix, not another guess

Implemented from `narrow-passage-navigation-fix-brief.md` (project root) -
a formal brief covering four symptoms: the global planner hugging one wall
in open aisles instead of the centreline; the robot still getting
permanently stuck near inflation boundaries inside passages even after the
earlier 0.12/14.0 fix; generally poor driving quality; and
`collision_monitor` losing its lifecycle bond heartbeat and taking the
whole stack down during a recovery-loop storm. Four architecture decisions,
implemented in full:

**Decision A - inflation is now computed from the real map, not eyeballed.**
This project had twice picked `(inflation_radius, cost_scaling_factor)` by
visual inspection alone and gotten burned both times - too small caused the
zero-margin flicker/deadlock documented above; too large looked visually
heavy. Neither attempt asked the one question that actually explains the
wall-hugging symptom: with a thin inflation band, the cost field is flat
zero across the vast majority of every open aisle, so `SmacPlanner2D` (a
cost-aware search) degenerates to a near-pure shortest-distance search with
no reason to stay centred - a documented Nav2 failure mode (Tuning Guide's
"Inflation Potential Fields" section; tracked upstream as
navigation2#4542), not something specific to this project.

`amr_navigation/scripts/calibrate_inflation.py` (new) answers this from the
map itself: distance-transforms `room_map_hires.pgm`, skeletonizes free
space into the map's own medial axis, splits the skeleton into branches at
its junctions, and classifies each branch as a PASSAGE (a width local
minimum between two junctions) or an AISLE (a sustained wide run) - purely
from geometry, with the layout drawing's known figures (353-460mm) used
only as an after-the-fact sanity check, not as an input. Two real bugs were
found and fixed *while building the script itself*, both worth remembering
for any future skeleton-based map analysis:
  - A branch's own ENDPOINT pixels sit at junctions (often near a wall
    corner where several branches meet) and their distance-transform value
    reflects that corner, not the corridor the branch runs through - an
    untrimmed pass reported branches down to 50mm wide, contradicting a
    fact already established elsewhere in this project (the earlier
    connectivity check showing every cell with clearance > robot_radius
    forms ONE connected region, meaning no real constriction here can be
    narrower than ~2x robot_radius = 0.20m). Fixed by trimming a few pixels
    off each branch end before taking its width minimum, and by requiring
    both of a branch's endpoints to be true junctions (degree >= 3) rather
    than a dead-end - a dead-end branch is a skeleton spur toward a convex
    bump (a round obstacle in this map), not a through-passage, and reads
    as falsely narrow for an unrelated reason.
  - A cross-section's perpendicular walk (used to check aisle-centering)
    was bounded by a large fixed pixel count, which let it wander out of
    the intended corridor into unrelated open floor space whenever a
    branch bent or the locally-estimated tangent was imprecise - one
    walked 1.8m from centre through an adjacent room. Fixed by bounding
    the walk to roughly 1.8x the point's own distance-transform value
    (its true distance to the nearest wall) instead of a generic constant.

Script output against this map's real geometry: the widest aisle segments
reach ~1.15m, requiring `inflation_radius` >= ~0.70m for both walls'
gradients to meet at the centre (coverage measured climbing
0%@<=0.35 -> 20%@0.40 -> 53%@0.50 -> 80%@0.60 -> 100%@0.70 as radius
increased - 0.70 is the smallest radius reaching full coverage, not a
round number picked by hand). At `cost_scaling_factor: 20.0` (the
steepest tested), cost decays from 253 at the inscribed edge to under
6/253 by 30cm out - despite the larger radius NUMBER, the visually
perceptible band is thinner than the previously-rejected 0.25/8.0 setting
(which stayed above 75/253 all the way to its own edge). The script's own
bridge-finding pass (Tarjan's algorithm on the skeleton's junction graph)
confirmed every real passage in this map (300-364mm; anything below
2x robot_radius is a wall-thickness/doorframe artifact no route ever
crosses, and is excluded as irrelevant) is single-entrance - no alternate
route exists anywhere for the planner to wrongly prefer - so
`planner_server.GridBased.cost_travel_multiplier` was raised `1.0 -> 1.3`
safely, and at the chosen inflation setting the narrowest real passage's
centreline cost is 93.7/253, comfortably short of the 253 cutoff that
would make `SmacPlanner2D` refuse it.

**Global and local costmaps now deliberately DIVERGE** (previously kept
identical on purpose - see "Robot stalled completely" entry above for why
that convention existed): `global_costmap.inflation_layer` gets the full
computed `inflation_radius: 0.70` / `cost_scaling_factor: 20.0` pair, since
aisle-centring is what `SmacPlanner2D` (which reads only the global
costmap) needs to fix. `local_costmap.inflation_layer` stays at a much
smaller `inflation_radius: 0.25` (same `cost_scaling_factor: 20.0`) for two
reasons: MPPI doesn't need its own large potential field to know where to
drive, since `PathAlignCritic`/`PathFollowCritic` already pull it onto the
path the global planner decided; and `InflationLayer`'s own per-obstacle
work scales with the SQUARE of the inflation-radius-in-cells, so blindly
copying 0.70 to the local costmap would have been roughly a 6x per-cycle
compute increase on the exact single-threaded executor already
diagnosed as this project's real bottleneck once before (see "Control
loop's real rate ceiling" entry). Confirmed this divergence doesn't
reintroduce the local/global disagreement `PathAlignCritic`'s
`max_path_occupancy_ratio` fix addressed: with the same scaling factor and
the same 253 inscribed core, local's smaller cutoff means its cost at any
given distance is always <= global's cost there (it just reaches zero
sooner) - local can never rate a point MORE forbidding than global already
did.

**Decision B - recovery behaviours now check a genuinely different,
less-paranoid costmap.** `Spin`/`BackUp`/`DriveOnHeading` were, until now,
collision-checked via `CostmapTopicCollisionChecker` against
`local_costmap/costmap_raw` - the SAME padded field MPPI/normal tracking
just failed against. When the robot's own pose reads as high-cost there
(the exact scenario in the "un-recoverable spin" deadlock, and in the
newer "flush against a wall" deadlock both documented above), every
recovery behaviour inherits that same poisoned reading and rejects itself
as unsafe on its first attempt too - there was no independent fallback
surface to actually recover onto. Confirmed real, configurable parameters
exist for this (`strings` on `libbehavior_server_core.so`:
`costmap_topic`/`footprint_topic`, defaulting to
`local_costmap/costmap_raw`/`local_costmap/published_footprint`) and a
real standalone costmap executable exists to point them at
(`ros2 pkg executables nav2_costmap_2d` lists `nav2_costmap_2d` itself - a
thin lifecycle-node wrapper around `Costmap2DROS` built for exactly this).

Stood up `recovery_costmap` as a new, separate lifecycle node
(`navigation.launch.py`, added to `lifecycle_manager_navigation`'s
`node_names`) with its own minimal costmap config (`obstacle_layer` +
`inflation_layer` only, no `static_layer`/`range_layer`, same rationale as
`local_costmap`), reusing this project's own already-vetted
`inflation_radius: 0.12` / `cost_scaling_factor: 14.0` pair (the value
`local_costmap` itself ran on safely before Decision A moved it to 0.25) -
deliberately NOT equal to `robot_radius`, since that exact equality is
what caused the original flicker/deadlock this whole change is meant to
route around. `behavior_server.costmap_topic`/`footprint_topic` now point
at `recovery_costmap/costmap_raw`/`recovery_costmap/published_footprint`
instead of the defaults. Left `simulate_ahead_time` at its existing `1.0`
rather than reverting it further - the brief's own concern (a longer
ahead-check window making self-rejection MORE likely) applied when this
window was checked against the same padded field recovery just failed
against; against `recovery_costmap`'s much-less-padded field there is
meaningfully more room before a cell reads as lethal - but this
interaction is flagged as something to watch during live verification, not
asserted as fully resolved by reasoning alone.

**Decision C - `collision_monitor`'s `CollisionSlow` no longer fires for
merely being inside a passage.** At the old `radius: 0.16`, this zone was
- by its own prior comment - active essentially 100% of the time inside
every passage in this layout, compounding continuously with MPPI's own
graduated caution (`CostCritic`/`ConstraintCritic`) reading the exact same
costmap the global plan was routed through. Two independent caution layers
stacking at 100% duty cycle in the same space is redundant, not additive
safety, and had already needed one compensating fix this project
(`slowdown_ratio` 0.3 -> 0.6) when the compounded cut collapsed velocity
below `controller_server.min_x_velocity_threshold`. Shrunk to `radius:
0.13`, computed (not eyeballed) from `calibrate_inflation.py`'s own
passage detection: the narrowest genuine passage in this map is 300mm
(150mm centre-to-wall at perfect centring), so 0.13 sits 2cm below that -
`CollisionSlow` no longer fires for nominally-tight-but-well-centred
passage transit, only for a genuinely closer-than-expected reading (drift,
an actual obstacle) in either a passage or the open aisles - while staying
1.5cm above `CollisionStop`'s untouched `0.115` last-resort ring.

**Decision D - hardening independent of whether A-C fully eliminate the
deadlock.** `maneuver_telemetry_node.py` already only logged on state
change, but a recovery-loop storm thrashes `classify()` between states
almost every `/cmd_vel_raw` message - exactly the scenario this brief
exists to fix - turning that into a near-every-cycle flood; added a 0.3s
minimum interval between log lines (state is still reported, just not
faster than that). Confirmed, via `strings` on the correct library
(`libnav2_lifecycle_manager_core.so` - not the same-sounding
`liblifecycle_manager_app.so`, which doesn't exist on this build), the
answer to what happens after a bond failure: `attempt_respawn_reconnection`
is a real parameter and WITHOUT it a bond failure is unrecoverable by
design (the library's own log strings are literally "CRITICAL FAILURE...
Shutting down related nodes" with no automatic bring-up path) - nav2
stayed down until manually relaunched. Enabled `attempt_respawn_reconnection:
true` with `bond_respawn_max_duration: 10.0` in `lifecycle_manager_navigation`'s
params, so a server that recovers within 10s is picked back up
automatically ("Successfully re-established connections from server
respawns, starting back up") instead of requiring a manual relaunch.
`bond_timeout` left at Nav2's own 4.0s default deliberately - the real
fix for the underlying trigger is Decisions A-C not letting the
executor-starvation storm happen at all; this is the safety net for if it
ever does anyway. Confirmed (same `strings` method) neither
`controller_server` nor `collision_monitor` has a configurable
multi-threaded executor on this Humble build - `collision_monitor` already
runs as its own OS process (not sharing `controller_server`'s executor),
so the only remaining lever for genuine CPU starvation would be an
OS-level scheduling priority change, which was deliberately NOT applied
without being asked (a system-level change beyond a config/code edit,
with its own side-effect risk) - noted here as the next thing to reach for
if `top`/`htop` during a live deadlock shows genuine CPU contention rather
than the single-executor starvation pattern already fixed.

**Verification status, honestly:** every number above was computed from
the real map and validated by re-deriving it two different ways where a
first pass looked suspicious (the branch-width and cross-section bugs
above were both caught this way, not assumed correct on the first run),
and the full config was checked for internal consistency (YAML parses,
`recovery_costmap` block present, `behavior_server` points at it,
`model_dt == 1/controller_frequency` still holds). What has NOT yet
happened is a live run against the actual simulation - this session had no
`ros2 node list` access at implementation time (checked, empty, more than
once) - so the brief's own acceptance criteria (a goal into a narrow-room
target completing without `RECOVER_BACKUP`/`IN_PLACE_SPIN` more than once
or twice transiently across 5 repeated attempts; `collision_monitor`'s
bond heartbeat never lapsing during a deliberately provoked deadlock) are
implemented against but not yet confirmed live. Treat this exactly like
the project's own prior "single successful run isn't sufficient
confirmation" standard - a fresh multi-attempt test is the natural next
step, not an afterthought.

## The standalone nav2_costmap_2d executable cannot be renamed - live-confirmed the hard way

The recovery_costmap node above was first launched with
`name='recovery_costmap'` in `navigation.launch.py`, matched by a
`recovery_costmap: recovery_costmap: ros__parameters:` block in
`nav2_params.yaml` and a bare `'recovery_costmap'` entry in
`lifecycle_manager_navigation`'s `node_names` - a reasonable-looking setup
that hung the ENTIRE navigation bringup forever on first real launch:
`lifecycle_manager` logged `Waiting for service
recovery_costmap/get_state...` on a 2-second retry loop indefinitely, and
because it brings nodes up in `node_names` order and won't proceed past
one it can't reach, everything listed after it (`behavior_server`,
`bt_navigator`, `velocity_smoother`, `collision_monitor`) never came up
either - a single wrong name took down the whole stack, not just the one
new node.

Root-caused live, with actual ROS graph access (`ros2 node list`) rather
than reasoning about it in the abstract: the node was really running, but
had registered itself as `/costmap/costmap`, not `/recovery_costmap`. This
was confirmed to be an unconditional, non-overridable limitation of the
`nav2_costmap_2d` executable itself, not a mistake in this project's
launch arguments - tested directly with
`ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args -r
__ns:=/recovery_costmap` (a completely separate, ad-hoc process, not this
project's own launch file), and it STILL reported as `/costmap/costmap` in
`ros2 node list`. Both a launch-level `name=` override and a ROS
`--ros-args -r __ns:=` namespace remap were tried and both were silently
ignored - `Costmap2DROS`'s internal construction for this executable
hardcodes its own name AND uses that same string as its own namespace
(explaining why every reference to it - including the embedded
`local_costmap`/`global_costmap` elsewhere in this same file - shows the
same doubled `<name>/<name>` pattern: that doubling is `Costmap2DROS`'s
own universal self-namespacing convention, not something specific to the
standalone executable).

Fixed by giving up on renaming it and wiring everything else to the real,
confirmed identity instead: `nav2_params.yaml`'s block became `costmap:
costmap: ros__parameters:`, `behavior_server.costmap_topic`/
`footprint_topic` became `costmap/costmap/costmap_raw`/
`costmap/costmap/published_footprint`, and `lifecycle_manager_navigation`'s
`node_names` entry became `costmap/costmap`. The launch file's
`name='recovery_costmap'` was left in place as a cosmetic label only (it
does appear in `ros2 node list`-adjacent tooling and log lines, just not
in the node's REAL reported identity or its topics/services) with a
comment explaining it's cosmetic, so a future reader doesn't assume it
controls anything functional.

Lesson: for a node type this project has never launched standalone before
(as opposed to `local_costmap`/`global_costmap`, always embedded here),
verify its real reported identity live (`ros2 node list`, `ros2 node
info`) BEFORE wiring three other files against an assumed name - a launch
`name=` field and a `--ros-args` remap are not guaranteed to control a
node's actual identity, particularly for a thin standalone wrapper around
a class (`Costmap2DROS`) that does its own internal node construction
rather than relying on the ROS launch system's usual name-resolution path.

## Two more live-diagnosed bugs in the same feature, then the computed inflation value was reverted by request

Two real, separate bugs surfaced on the first actual relaunch after the
above naming fix, both caught with genuine live `ros2 lifecycle`/`ros2
node` introspection rather than assumed fixed:

1. **`bond_timeout` (4.0s, Nav2's stock default) was too tight for THIS
   bringup specifically.** `costmap/costmap` genuinely reached `active`
   (confirmed via `ros2 lifecycle get /costmap/costmap` immediately after
   the failure) - its bond simply wasn't detected inside the 4.0s window
   lifecycle_manager waited before giving up ("Server costmap/costmap was
   unable to be reached after 4.00s by bond"), most likely because this
   added a FIFTH lifecycle node on top of an already-running Webots + RViz
   + full stack bringup, and the extra contention pushed its first bond
   handshake past the window. Because lifecycle_manager aborts the ENTIRE
   bringup the instant one node in `node_names` is late - not just that
   one node - everything listed after it (`behavior_server`,
   `bt_navigator`, `velocity_smoother`, `collision_monitor`) was left
   stuck `inactive` even though nothing was actually wrong with any of
   them (confirmed via `ros2 lifecycle get` on each). Raised to `8.0s` -
   a bringup-time contention issue is a different thing from the
   steady-state heartbeat-loss scenario this same parameter also governs
   (a genuine executor-starvation storm, which Decisions A-C address,
   produces gaps measured in many seconds to indefinite, not a difference
   of ~1s), so this doesn't materially weaken the runtime safety net.

2. **The computed `inflation_radius: 0.70` / `cost_scaling_factor: 20.0`
   pair, while functionally verified (100% aisle-gradient coverage, no
   passage newly blocked, confirmed via calibrate_inflation.py), was
   rejected on sight in RViz before a live navigation test could show
   whether the trade-off was worth it.** Shown the concrete trade-off
   directly (the wall-hugging symptom this whole brief exists to fix is
   expected to return at a smaller radius, per the Nav2 Tuning Guide's own
   documented failure mode) and asked explicitly rather than guessed at
   again - the answer was to revert anyway, accepting that consequence.
   Reverted global_costmap AND local_costmap to `0.12`/`14.0` (matching
   each other again, restoring this file's original global/local-identical
   convention that Decision A had deliberately broken) and
   `costmap`/`costmap` (the recovery costmap) already ran that exact pair.
   `cost_travel_multiplier` was NOT reverted (stayed at `1.3`) - its own
   justification (every real passage in this map is single-entrance, so a
   higher multiplier can't cause detour-seeking) doesn't depend on the
   inflation radius value at all.

The computed 0.70/20.0 pair is not lost - it's documented in the
"Open-aisle wall-hugging..." entry above, along with the full derivation,
in case the wall-hugging behaviour is confirmed to return and revisiting
it (or a value between the two, re-run through the calibration script
with a different acceptance threshold) becomes worth it. This is a
recorded, deliberate trade-off, not an unresolved bug - don't re-raise
inflation size as an open problem without first checking whether this
entry already covers the exact trade-off in question.

## Final inflation tuning state: global 0.15/10.0, local 0.11/25.0

**Current asymmetric configuration** (Sep 2026): global_costmap inflation
`0.15m` radius / `10.0` scaling, local_costmap `0.11m` / `25.0`. Both soft
cost only — the hard inscribed core blocking navigation is `robot_radius:
0.10`, unchanged. Global band is 5cm beyond inscribed (2 grid cells on
0.025m), rendering as a thin line instead of the large soft halo seen at
0.30+. Local band is 1cm (1-2 cells on 0.05m) — at its floor before
triggering grid-indistinguishable-from-equal flicker again.

Trade-off: with a 5cm band, SmacPlanner2D has no centering gradient in
aisles wider than ~600mm and degenerates to shortest-distance search,
expected to cause wall-hugging in the widest open areas again. Known
documented failure mode. Re-raises inflation toward 0.30+ or 0.55+ if
hugging returns and matters; the full history and reasoning above covers
the decision space completely.

The soft-cost band is a visualization/tuning parameter only. It has never
blocked the robot — it cannot make the robot fit where it didn't before.
What blocks is the hard inscribed radius (robot_radius: 0.10). Reducing
soft cost only makes the RViz display lighter, not the robot smaller.

Current MPPI tuning (temperature: 0.35, vx_std: 0.10, wz_std: 0.15) and
planner cost_travel_multiplier: 1.5 are the levers for passage-entry
behavior, not inflation_radius itself.

---

## Station-1 traversal session (2026-09-25) — what changed and why

**Robot is a TurtleBot3 Burger (differential drive)**, not mecanum; older sections above that talk about
mecanum/EKF/RF2O describe the earlier robot. Current odom->base_link = diffdrive wheel odometry
(`enable_odom_tf: true`); RF2O is not launched because the Webots /scan has a negative
`angle_increment` and RF2O mirrors it. `ekf.yaml`/`sensor_wrapper_node.py` are currently unused.

**Map frame == Webots world frame.** `maps/room_map_world.{pgm,yaml}` is rasterised from
`simulation.wbt` by `maps/make_world_map.py` (the SLAM map was 3-8 % undersized and lacked the Station-1
wall and pillar). AMCL initial pose = robot spawn pose in the world file (1.27, 0.38, yaw pi).
**LIF -> map** = 180 deg rotation: `map = (1.51 - lif_x, 1.25 - lif_y)` (all 22 LIF nodes land in free space).
Physical Station 1 = LIF nodes 18 (enter) -> 10 (inside) -> 17 (exit).

**tf2 deadlock (root cause of "goals SUCCEED instantly, robot doesn't move")**: stock tf2_ros 0.25.23
`Buffer::waitForTransform` holds `timer_to_request_map_mutex_` while calling `addTransformableRequest()`,
and `BufferCore::testTransformableRequests()` holds the requests mutex while its callback takes the timer
mutex -> lock-order inversion. In controller_server the LaserScan MessageFilter (costmap obstacle layer)
hits this within seconds of activation; the TF listener thread then freezes forever. Diagnosed with gdb
(thread stacks). Fixed by a patched copy of tf2_ros in this workspace (`src/tf2_ros`, overlay - source
`install/setup.bash` of this workspace AFTER the ROS underlay). Symptom to check:
`/local_costmap/published_footprint` header stamps stop advancing.

**Controller**: Regulated Pure Pursuit is the active `FollowPath`; MPPI is kept as `FollowPathMPPI`
(it stalled at ~0.005 m/s in the 0.35 m station gaps). Goals are position-only
(`yaw_goal_tolerance 3.14`, xy 0.07). Collision monitor: lidar added as a source; the stop zone is the
APPROACH model (direction-aware) so recovery can back away from a wall.

**Station run** (`scripts/fleet_manager.py`, service `/fms/station_1`): approach point -> node 18 -> node 10 ->
node 17 -> exit-clear point (0.35 m beyond the mouth, direction derived from the LIF nodes), so the robot
actually leaves the station.
