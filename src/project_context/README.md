# AMR Project — Context Index

This folder exists so a new agent (or a human) picking up this workspace
cold can get oriented without re-deriving hours of hard-won debugging.
Read these in order the first time; after that, use them as reference.

1. **architecture.md** — packages, TF tree, node/data flow, topic map.
2. **decisions-and-gotchas.md** — the non-obvious choices and bugs that
   will bite you again if you don't know about them. Read this before
   touching `controllers.yaml`, `ekf.yaml`, `amcl.yaml`, or any sensor
   xacro.
3. **proto-regeneration.md** — the manual pipeline that turns the URDF
   into the Webots PROTO. This is NOT part of `colcon build` — if you
   change any xacro file under `amr_description/urdf/`, you must run this
   pipeline by hand or your changes will never reach Webots.
4. **launch-reference.md** — what each launch file starts, in what order,
   and which ones depend on which.

## What this project is

TurtleBot3 Burger (2WD differential drive + caster), simulated in Webots
R2025a on ROS 2 Humble, navigating this project's own `simulation.wbt`
world (not the mecanum ROSMASTER-X3 platform this project started as —
see decisions-and-gotchas.md's "Drivetrain: 2WD + caster" and TurtleBot3
sections for that transition). Three packages:

- **`amr_description`** — URDF/xacro, meshes, RViz config. Single source
  of truth for the robot's physical model. Builds for two targets from
  the same xacro tree via the `use_webots` arg (see architecture.md).
- **`amr_simulation`** — Webots world/PROTO, `ros2_control` bring-up,
  twist_mux, teleop. This is the "make it move in Webots" package.
- **`amr_navigation`** — the full Nav2 autonomy stack: AMCL localization,
  SmacPlanner2D global planning, MPPI local control, rf2o_laser_odometry
  as the odom->base_link TF authority, and nav2_collision_monitor as a
  last, source-agnostic safety net driven by four ultrasonic proximity
  sensors. This is the "make it know where it is and drive itself there"
  package. (`fleet_control.launch.py` here is currently an empty
  placeholder for future work, not implemented yet — don't assume it does
  anything.)

## Current status (as of this writing)

Working: gravity/physics, differential drive, Lidar/IMU sensors, four
ultrasonic proximity sensors (mount geometry corrected — see "Known-open
items" — covering LIDAR's own minimum-range blind spot and feeding
`nav2_collision_monitor` directly, no longer the costmap — see below),
rf2o scan-matched odom->base_link TF, AMCL localization, SmacPlanner2D
global planning, `controller_server` sustaining its full configured
10Hz control loop with zero missed-rate warnings (live-verified — see
decisions-and-gotchas.md's "Control loop's real rate ceiling" entry),
MPPI local control tuned to this robot's own velocity limits rather than
Nav2's stock example values (sampling noise, `PathAlignCritic`'s
kill-switch threshold — see decisions-and-gotchas.md), collision_monitor
as an independent stop/slowdown safety layer, goal completion with a
forgiving-but-real tolerance, and half-speed rotation vs. a raised
0.45m/s linear speed.

Navigating this layout's 353mm/324mm narrow passages has gone through
several rounds of live-diagnosed fixes this session (control-loop
starvation, MPPI sampling noise dominating over signal, a critic
kill-switch, an inflation zero-margin deadlock) — each was verified
individually via live telemetry, but a full fresh end-to-end run under
ALL of the current settings together has not yet been confirmed by the
user as of this writing. Don't assume narrow-passage navigation is fully
solved without checking for a more recent update to this section.

Known-open items — check these before assuming they're fixed:
- `predict_to_current_time`-style crash history exists for
  `robot_localization`-based EKF (this project no longer runs `ekf_node`
  — rf2o is the TF authority now — but the underlying
  `robot_localization` 3.5.4-1jammy crash is worth knowing if EKF is ever
  reintroduced). See decisions-and-gotchas.md.
- RViz's `Map` display crashes with the default `map` color scheme on
  this machine's Mesa Intel driver (GLSL shader link failure). Must use
  `costmap` color scheme instead. Unconfirmed whether this is
  machine-specific or a wider Mesa/Ogre bug.
- `maps/room_map_hires.yaml` (0.025m/px) is `navigation.launch.py`'s
  default GLOBAL map, generated from `maps/room_map.yaml` (0.05m/px,
  untouched, still the fallback if the launch arg is pointed back at it)
  — see decisions-and-gotchas.md if either ever needs regenerating.
  `local_costmap.resolution` is deliberately still 0.05 (NOT matched to
  the hires map) — that fine resolution was only ever needed for
  SmacPlanner2D's global grid-search, and matching it locally overloaded
  controller_server's single-threaded executor (see decisions-and-
  gotchas.md's "Robot stalled completely" entry) — don't "fix" this
  mismatch without rereading that entry first.
- `dwb_core::DWBLocalPlanner` and `nav2_regulated_pure_pursuit_controller`
  are both preserved inactive in `nav2_params.yaml` as `FollowPathDWB` /
  `FollowPathRPP` — `nav2_mppi_controller::MPPIController` is the active
  `FollowPath` now. Swap block names to revert to either.
- `behavior_server`'s `max_rotational_vel`/`min_rotational_vel`/
  `rotational_acc_lim`/`simulate_ahead_time` live at the TOP LEVEL of its
  `ros__parameters`, not nested under `spin:`/`backup:` — confirmed live
  via `ros2 param list /behavior_server` after a nested copy sat silently
  ignored for most of this project's history. See decisions-and-gotchas.md
  before assuming any behavior_server parameter's nesting from naming
  convention alone; verify against the live node instead.
- No configurable multi-threaded executor exists for this Humble build's
  `nav2_controller`/`controller_server` (confirmed via `strings` — no
  `executor_type`/`num_threads` parameter). Per-cycle compute (costmap
  resolution, MPPI `batch_size`/`time_steps`, costmap layer count) is the
  only lever for "the control loop can't sustain its configured rate" on
  this stack.
- `controller_frequency` is `10.0` (not the stock `20.0`) and MPPI's
  `FollowPath.model_dt` (`0.1`) MUST always equal
  `1/controller_frequency` — they were live-diagnosed out of sync for
  much of this session (a `20Hz` config actually running at a measured
  `4.33Hz`, with `model_dt` still at `0.05`) and it silently made MPPI
  over/undershoot every planning window. If `controller_frequency` is
  ever changed, `model_dt` must change with it in the same edit. See
  decisions-and-gotchas.md's "Control loop's real rate ceiling" and
  "MPPI's model_dt must equal 1/controller_frequency" entries.
- `local_costmap.plugins` deliberately does NOT include `range_layer`
  (the block is still defined in `nav2_params.yaml`, just unused) — the
  four ultrasonic sensors sit only 0.10m from `base_link` with a
  0.01–0.08m range, so any mark they produce lands inside the robot's own
  inflation radius and can paint the robot's own centre cell lethal. The
  sensors still feed `nav2_collision_monitor` directly, which is the
  correct consumer for a sensor this short-range. Re-adding `range_layer`
  to the costmap without rereading decisions-and-gotchas.md will
  reintroduce a real deadlock, not just noise.
- `inflation_radius` ASYMMETRIC: global `0.15`, local `0.11` (both must be
  > `robot_radius` `0.10`). Global is soft cost only — 5cm band beyond
  inscribed, which is 2 grid cells on 0.025m and renders as a thin line in
  RViz instead of a large halo. Local is at its floor: 1cm band, which on
  0.05m grid cannot be smaller without re-introducing the grid-indistinguishable-
  from-equal flicker that defeats Spin/BackUp recovery. See decisions-
  and-gotchas.md for full history of this tuning and why the split exists.
- MPPI's `vx_std`/`wz_std`/`temperature` are NOT Nav2's stock example
  values (those are sized for a 0.5/1.9 m/s robot; this one runs
  0.45/0.6). Check them as a fraction of this project's actual
  `vx_max`/`wz_max` (~20–25% is the working range found here) before
  assuming a copied default is safe. See decisions-and-gotchas.md.
- `FollowPath.PathAlignCritic.max_path_occupancy_ratio` has a real
  kill-switch behavior, not just a weight — past this ratio the critic
  stops scoring entirely for that cycle. Currently `0.5` (raised from an
  over-strict `0.05` that was silently disabling it inside every narrow
  passage). See decisions-and-gotchas.md.
- The four ultrasonic sensors' `translation`s in `simulation.wbt` are
  written relative to `TurtleBot3Burger.extensionSlot`'s own origin,
  which is NOT `base_link` — it's offset to `(-0.03, 0, 0.153)` (the
  lidar mount point). A translation intended as "X m from centre" must
  be back-solved as `(desired base_link pose - slot origin)`; verify any
  change with `ros2 run tf2_ros tf2_echo base_link <sonar_frame>`, never
  trust the raw number written in the world file. See decisions-and-
  gotchas.md's "Ultrasonic sensors were mounted in the wrong frame" entry
  — this was live-diagnosed after the sensors were found 3cm back and
  15cm too high.
- `global_costmap`/`local_costmap`/the recovery costmap (see below) all run
  `inflation_layer: 0.12`/`14.0` again — a *computed* `0.70`/`20.0` pair
  for `global_costmap` (from `amr_navigation/scripts/calibrate_inflation.py`,
  which fixes the global planner hugging one wall instead of centring in
  open aisles) was tried, verified functionally sound, and then reverted
  by explicit request after being shown the trade-off directly — the
  wall-hugging symptom it fixed is therefore expected to be back. This is
  a deliberate, recorded trade-off (decisions-and-gotchas.md's "Open-aisle
  wall-hugging..." and the follow-up entry after it), not an unresolved
  bug — don't re-derive or re-litigate it without reading those first; the
  computed value is sitting there ready to reapply if ever wanted.
- A standalone recovery costmap (own lifecycle node) now exists solely for
  `behavior_server`'s `costmap_topic`/`footprint_topic` — `Spin`/`BackUp`/
  `DriveOnHeading` no longer check the same costmap normal tracking uses.
  **Its real node identity is `costmap/costmap`, NOT `recovery_costmap`** —
  the standalone `nav2_costmap_2d` executable hardcodes this regardless of
  the launch `name=` field, live-confirmed the hard way (see decisions-
  and-gotchas.md's "The standalone nav2_costmap_2d executable cannot be
  renamed" entry) after a first attempt using the nicer name hung the
  *entire* nav2 bringup indefinitely. Every reference to it — 
  `lifecycle_manager_navigation`'s `node_names`, `behavior_server`'s
  `costmap_topic`/`footprint_topic`, the params file's own top-level key —
  is deliberately written as `costmap/costmap`; don't rename any one of
  them without the others.
- `lifecycle_manager_navigation`'s `bond_timeout` is `8.0`, not Nav2's
  stock `4.0` — the recovery costmap above is a fifth lifecycle node added
  to bringup, and 4.0s was measured too tight for its first bond handshake
  under full Webots+RViz+stack load, aborting the ENTIRE bringup (not just
  that one node) even though the node itself was fine. See decisions-and-
  gotchas.md before lowering this back down.


Latest: see `decisions-and-gotchas.md` -> "Full multi-station mission (2026-09-26)" for the station routes, `/fms/*` services and the boot/stop scripts in `tools/`.
