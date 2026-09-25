# Launch File Reference

## `amr_simulation/launch/sim.launch.py`
**Always run this first.** Starts: `WebotsLauncher` (+ its Ros2Supervisor),
`robot_state_publisher` (publishing the flattened `use_webots:=true` URDF
directly — see note below), `WebotsController` (`amr_robot`),
`joint_state_broadcaster` + `mecanum_drive_controller` spawners (gated on
`WaitForControllerConnection` so they don't race Webots' connection),
`rviz2` (loads `amr_description/rviz/viewport_config.rviz`), and
`ekf_node` (fuses odom+IMU into `odom -> base_link` — merged in from the
now-deleted standalone `localization.launch.py`).

Args: `world` (default `simulation.wbt`), `mode` (default `realtime`),
`use_sim_time` (default `true`).

*Why `robot_state_publisher` gets the real URDF content directly instead
of `set_robot_state_publisher=True` on `WebotsController`:* the latter
makes Webots synthesize its own URDF from the live PROTO via
`wb_robot_get_urdf()`, which reproducibly produced a duplicate `base_link`
error for this robot's structure. Publishing our own pre-validated URDF
avoids that path entirely.

## `amr_simulation/launch/mapping.launch.py`
Runs `slam_toolbox`'s `async_slam_toolbox_node` with
`amr_simulation/config/mapper_params_online_async.yaml`. Publishes
`map -> odom` and `/map` while you drive the robot around. **Mutually
exclusive with `amr_navigation/localization.launch.py`** — both publish
`map -> odom`. Run this to build a map, save it with
`ros2 run nav2_map_server map_saver_cli -f ~/ros/maps/room_map`, then
switch to AMCL for actual localization runs.

## `amr_simulation/launch/teleop.launch.py`
Documents the correct remap
(`/cmd_vel -> /mecanum_drive_controller/reference_unstamped`) but **will
not actually work run via `ros2 launch`** — see decisions-and-gotchas.md's
Teleop section. Drive with:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mecanum_drive_controller/reference_unstamped
```

## `amr_navigation/launch/localization.launch.py`
Static map + AMCL. Starts `nav2_map_server` (`map_server`), `nav2_amcl`
(`amcl`, params from `amr_navigation/config/amcl.yaml`), and
`nav2_lifecycle_manager` (`autostart: true`, manages both nodes into
Active state automatically — no manual lifecycle transitions needed).

Args: `map` (default `~/ros/maps/room_map.yaml`), `use_sim_time`
(default `true`).

AMCL's `initial_pose` is hardcoded in `amcl.yaml` to a verified map-frame
pose (not the Webots world-frame spawn translation — see
decisions-and-gotchas.md's AMCL section for why those two are expected to
differ) — no manual "2D Pose Estimate" click needed on a fresh launch, as
long as `initial_pose` is re-verified whenever the robot's real boot-time
position relative to the saved map changes.

## `amr_navigation/launch/navigation.launch.py` (Milestone 3)
Full Nav2 autonomy stack: `map_server`, `amcl`, `planner_server` (NavFn),
`controller_server` (DWB, holonomic-tuned for mecanum strafing), 
`behavior_server` (Spin/BackUp), `bt_navigator`, all under one
`nav2_lifecycle_manager` (`lifecycle_manager_navigation`, `autostart: true`).
All servers load `amr_navigation/config/nav2_params.yaml` (one consolidated
params file, including its own `amcl:` block kept in sync with the
standalone `amcl.yaml` — see decisions-and-gotchas.md's Nav2 stack
section for several plugin-name and parameter-name fixes made against the
real installed binaries).

Args: `map` (default `~/ros/maps/room_map.yaml`), `params_file` (default
`nav2_params.yaml`), `use_sim_time` (default `true`).

**Crucial remap**: `controller_server` publishes velocity commands on
`cmd_vel` by default — remapped in this launch file to
`/mecanum_drive_controller/reference_unstamped`, the only topic
`mecanum_drive_controller` actually subscribes to (see Controller stack
section). Without this remap Nav2 would compute paths and velocities that
never reach the wheels.

Superseded by this file for actual autonomous navigation runs;
`localization.launch.py` remains as a lighter-weight map+AMCL-only entry
point for localization testing without the full planner/controller stack.

## `amr_navigation/launch/fleet_control.launch.py`, `slam.launch.py`
**Currently empty (0 bytes).** Placeholders for future work, not
implemented. Don't assume these do anything until someone writes them.

## Typical run order

```
Terminal 1: ros2 launch amr_simulation sim.launch.py
Terminal 2 (mapping):          ros2 launch amr_simulation mapping.launch.py
        OR (localization only): ros2 launch amr_navigation localization.launch.py
        OR (full autonomy):     ros2 launch amr_navigation navigation.launch.py
Terminal 3: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mecanum_drive_controller/reference_unstamped
```
