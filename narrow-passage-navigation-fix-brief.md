# Implementation Brief: Open-Aisle Path Quality + Narrow-Passage Recovery Deadlock

**Audience:** Claude Code, working in this repo (`amr_description` / `amr_simulation` / `amr_navigation`).
**Do not re-litigate the architecture decisions below.** They were made after reading the full project history (`decisions-and-gotchas.md`, `architecture.md`, the live `nav2_params.yaml`) and cross-checking against Nav2's own maintainers' documentation and the path-planning literature (see References). Where a decision depends on a number that can only come from this project's actual map file, that is called out explicitly — write the script, run it, use its output, don't guess.

---

## 1. Symptoms reported (live testing, TurtleBot3 Burger, this project's `simulation.wbt` / `room_map_hires`)

1. **Global planner doesn't take the most open route.** In wide aisle sections where the full width is free, `SmacPlanner2D` produces a path that runs close to and parallel to one wall instead of down the middle, even though centerline travel is available.
2. **Once the robot is inside a narrow passage and near the inflation boundary, it cannot get out.** Matches this project's own previously-documented `RECOVER_BACKUP <-> IN_PLACE_SPIN <-> TRANSITIONING` loop, ending in `Failed to make progress` / `[follow_path] Aborting handle`. **This is still happening live** with the current `inflation_radius: 0.12` / `cost_scaling_factor: 14.0` settings — the earlier fix for this (see `decisions-and-gotchas.md`, "inflation_radius == robot_radius is not a valid tuning value") reduced but did not eliminate it.
3. **General driving quality is poor** (jitter, hesitation) — largely a downstream symptom of (1) and (2): a controller fighting a path that hugs inflation, or a robot stuck in a doorway, cannot drive smoothly regardless of MPPI tuning.
4. **New finding, not yet in `decisions-and-gotchas.md`:** during the (2) recovery loop, `collision_monitor` stopped publishing its lifecycle bond heartbeat, `lifecycle_manager_navigation` declared it down after the default 4000ms `bond_timeout`, and reset the whole nav2 stack. This is a real, separate hazard: **the emergency-stop layer is dying at exactly the moment the robot is most likely to need it.**

---

## 2. Root cause analysis

### 2.1 Why the planner hugs walls in open space

The project's own `global_costmap.inflation_layer` is currently `inflation_radius: 0.12`, `cost_scaling_factor: 14.0` — set deliberately thin to keep 353mm doorways traversable. `planner_server.GridBased.cost_travel_multiplier` is `1.0` for the same reason. Combined, this means the costmap has **zero cost gradient anywhere more than 12cm from a wall** — the vast majority of every open aisle. `SmacPlanner2D` is a cost-aware A*/Dijkstra search; with no cost differentiation across most of the free space, it degenerates to a near-pure shortest-distance search, and shortest-distance paths have no reason to stay centered — they follow whatever the grid search's tie-breaking happens to prefer, which is exactly the "hugs one wall" behavior reported. This is a known, named Nav2 failure mode, not a bug specific to this project:

- Nav2's own Tuning Guide states directly that a small inflation radius is a common mistake, that planners including Smac Planner "really want to look for a smooth potential field rather than wide open 0-cost spaces in order to stay in the middle of spaces," and that for halls and aisles specifically, the inflation radius and cost scale should be raised to create a real potential field across the corridor width, not just a thin band at the wall.
- A tracked Nav2 GitHub issue (#4542) documents the same Smac-family planner producing wobbly, off-center paths specifically in open, near-0-cost regions of the costmap.
- This corridor-centering behavior is the textbook "maximum-clearance path" / generalized Voronoi diagram (GVD) result (LaValle, *Planning Algorithms*, §6.2; Blaer, "Robot Path Planning Using Generalized Voronoi Diagrams") — a path search that is cost-aware over a real clearance gradient approximates a GVD/maximum-clearance path; a path search over a flat cost field does not, regardless of planner.

**This project's earlier fix for the doorway-refusal bug (lowering `cost_travel_multiplier` and shrinking `inflation_radius` to almost nothing) solved doorway traversal by removing the very thing that also makes the planner prefer open space.** Both symptoms come from the same knob pushed in opposite directions. The fix is to stop treating this as one global number and make it a *computed, verified* choice (Decision A below) instead of another guess.

### 2.2 Why the robot still gets stuck near inflation, and why collision_monitor dies

This project's own diagnosis (see `decisions-and-gotchas.md`) already identified the mechanism: `Spin`/`BackUp`/`DriveOnHeading` are all collision-checked against the **local costmap**, using the same inscribed-radius/inflation logic as everything else. When ordinary AMCL/rf2o pose noise (several cm, already documented elsewhere in this project) pushes the robot's own footprint over the inscribed boundary inside a passage with only ~76.5mm of nominal clearance per side (353mm passage, 100mm robot radius), the robot's **current** pose reads as already-colliding, so every recovery behavior rejects itself on its first attempt, and MPPI sees forward/reverse/rotate all scoring near-equally badly. The fact that this is **still reproducing live** after the `inflation_radius: 0.10 -> 0.12` fix means that fix reduced the frequency but did not remove the underlying coupling: **recovery behaviors and normal path tracking share the exact same collision-check surface**, so there is no margin left to fall back on once normal tracking has already failed once.

Separately: `nav2_util::LifecycleNode` publishes a bond heartbeat every 0.1s, and `lifecycle_manager`'s default `bond_timeout` is 4.0s — matching the "4000 ms" in the observed log exactly. A >4s heartbeat gap this large is a classic executor-starvation signature, not a `collision_monitor`-specific logic bug: this project has already independently diagnosed the same "single-threaded executor starved by high-frequency work" pattern for `controller_server` (the `range_layer` costmap-marking incident). The recovery-loop storm generates a very high rate of telemetry logging and rapid state transitions at exactly the moment `collision_monitor` most needs to be responsive — a plausible, checkable link.

### 2.3 What does *not* need to change

No new sensors are needed for this. The four ultrasonics plus the LIDAR already give complete, accurate coverage of every wall and doorway in this map; the problem is in the cost field the planner searches over and in how the two independent safety layers (MPPI's own critics vs. `collision_monitor`) interact, not in what the robot can sense.

---

## 3. Architecture decisions (final — implement these)

**Decision A — Replace the guessed `(inflation_radius, cost_scaling_factor, cost_travel_multiplier)` triple with a computed one**, derived from a distance-transform of the actual map, verified against two explicit numeric criteria (open-aisle centering, doorway pass-ability) instead of visual inspection. Keep `SmacPlanner2D` — do not switch planners. (`nav2_theta_star_planner` is a documented alternative with an explicit "prefer middle of spaces" cost-weighting mode — note it in code comments as a fallback option, but only pursue it if Decision A, properly calibrated, still fails live verification.)

**Decision B — Decouple recovery-behavior collision checking from the same margin that is causing self-rejection.** Recovery behaviors need a genuinely different (less paranoid) collision surface than the one governing normal tracking, not just a slightly bigger version of the same one.

**Decision C — Reduce compounding caution inside known-tight passages.** `collision_monitor`'s `CollisionSlow` ring and MPPI's own `CostCritic`/`ConstraintCritic` currently apply graduated caution in the same physical space at the same time; this project has already had to raise `slowdown_ratio` once (0.3 -> 0.6) to stop it collapsing to zero velocity, and the live logs show it is still marginal. Make `collision_monitor` a true last-resort stop layer, and let MPPI's own critics own graduated speed control inside marked passages.

**Decision D — Harden `collision_monitor` and reduce log-driven executor load during recovery storms**, independent of whether A–C fully eliminate the deadlock.

---

## 4. Work items

### 4.1 New tool: `amr_navigation/scripts/calibrate_inflation.py`

Purpose: turn Decision A from a guess into a verified computation.

Requirements:
- Load `maps/room_map_hires.pgm` + `.yaml` (the 0.025m/px map already used by the global planner).
- Compute a Euclidean distance transform (`scipy.ndimage.distance_transform_edt`, in meters using the map resolution) giving each free cell's true distance to the nearest occupied cell.
- Implement Nav2's own inflation cost formula: for a cell at distance `d` from the nearest obstacle, with inscribed radius `r_i = robot_radius + footprint_padding` (currently `0.10 + 0.0`):
  - `d <= r_i` → cost `253` (hard, non-negotiable — this never changes regardless of `inflation_radius`)
  - `r_i < d <= inflation_radius` → cost `= 252 * exp(-cost_scaling_factor * (d - r_i)) + 1`
  - `d > inflation_radius` → cost `0`
- Auto-detect the doorway/passage locations and the open-aisle segments from the map itself (skeletonize the free-space mask — `skimage.morphology.skeletonize` — and find the skeleton's local-minimum-width points for passages vs. the wider stretches for aisles) rather than hardcoding the numbers read off the layout drawing (353/405/360/460mm) — those are useful as a sanity check, not as ground truth for the script.
- Grid-search `inflation_radius` in a plausible range (start `0.15` to `0.60` in `0.05` steps) and `cost_scaling_factor` in (`3.0` to `20.0` in steps of `1.0`), and for each pair report:
  1. **Aisle-centering check:** at several cross-sections through each detected open-aisle segment, is the cost minimum located within the map's own centerline tolerance (e.g. within 1-2 cells of the true medial axis), and is there a real, non-flat gradient across most of the aisle width (not just a thin band at each wall)?
  2. **Passage headroom check:** at each detected passage's narrowest cross-section, what is the cost at the centerline point? This must stay comfortably below whatever `cost_travel_multiplier` will be applied would make attractive to detour around — report it as a plain number, don't threshold it in the script.
- Since most of this project's passages are single-entrance rooms (AS1/AS2/AS3 — there is no alternate route into them), a real detour is only possible for routes that have another way around; the script should identify, per passage, whether an alternate route actually exists in the map topology (via a graph built from the skeleton) — this matters because a passage with no alternate route can tolerate a much higher `cost_travel_multiplier` than one that does.
- Output: a short report (print to stdout is fine) recommending one `(inflation_radius, cost_scaling_factor)` pair per costmap (global and local can differ) and a `cost_travel_multiplier`, plus the supporting numbers above. Use this output — not a guess — for 4.2.

### 4.2 Edit `amr_navigation/config/nav2_params.yaml`

- `global_costmap.global_costmap.inflation_layer.inflation_radius` / `.cost_scaling_factor`: set from 4.1's output. As a starting hypothesis for the script to test (not a final answer): something in the range `inflation_radius: 0.30–0.40`, `cost_scaling_factor: 8.0–12.0` is far more likely to create a real cross-aisle gradient than the current `0.12`/`14.0`, per the Nav2 Tuning Guide's explicit recommendation to size inflation to the corridor width, not to the doorway width.
- `local_costmap.local_costmap.inflation_layer`: keep in sync with the global values per this file's own established convention (see the existing comment block there), **except** intentionally give the local costmap a **slightly larger inscribed margin than the bare minimum** (i.e. don't let `local_costmap`'s effective margin drop to zero even where global's does) — the local costmap is what recovery behaviors and MPPI actually check against in real time, and it needs a little real slack to absorb ordinary AMCL/rf2o jitter without immediately reading the robot's own pose as lethal. Confirm any difference between the two doesn't reintroduce the "local costmap disagrees with the global plan it's following" issue this file's own comments already warn about.
- `planner_server.GridBased.cost_travel_multiplier`: set from 4.1's output, informed by the per-passage alternate-route analysis (single-entrance passages can take a higher value safely; passages with a real alternate route need it kept low enough not to trigger detour-seeking). Do not raise it uniformly without that check — that is exactly the failure this project already diagnosed once (`decisions-and-gotchas.md`, "Real root cause: costmap resolution, not inflation" section and the earlier `cost_travel_multiplier: 2.0 -> 1.0` change).

### 4.3 `collision_monitor` — Decision C

- In `amr_navigation/config/nav2_params.yaml`'s `collision_monitor` block: reduce `CollisionSlow`'s role to genuinely open areas only, or shrink it enough that it stops firing for the entire duration a robot spends inside any passage (currently, by this file's own comment, it is active essentially 100% of the time inside every passage in this layout, by design — that design is exactly what compounds with MPPI's own caution). Two ways to do this, pick whichever is simpler to verify live:
  1. Reuse the passage mask/skeleton from 4.1 to gate `CollisionSlow` off while the robot's pose is inside a marked passage (requires a small node or a `nav2_costmap_2d` filter — check `nav2_collision_monitor`'s real source for whether polygons can be conditionally enabled at runtime via a topic/service before building new infrastructure for this), **or**
  2. Simply reduce `CollisionSlow.radius` enough that it only fires meaningfully in the open aisles (where there's room to slow down productively) and let MPPI's own `CostCritic`/`ConstraintCritic` be the sole graduated-caution authority inside passages, since they already read the same costmap the global plan was computed against.
- Keep `CollisionStop`'s tight 0.115m ring exactly as configured — that is the genuine last-resort layer and should not be touched.

### 4.4 Recovery collision-checking — Decision B

- Check (via `ros2 param list /behavior_server` and the real installed `nav2_behaviors` source, per this project's own established verification standard) whether `behavior_server`'s `CostmapTopicCollisionChecker` can be pointed at a **separate costmap topic** with a smaller/zero inflation margin than the one MPPI uses for path tracking — i.e. collision-check recoveries against something closer to raw inscribed-radius-only geometry, not the same padded field that just caused the tracking failure. If that parameter exists, stand up a second, minimally-inflated local costmap instance (or a `PluginContainerLayer` combination, which this Nav2 version supports) for behavior_server to check against instead.
- If no such per-node costmap override exists in this installed version, implement a narrower fallback: when the robot is stuck (BT detects repeated recovery failure) **inside a marked passage** (reuse 4.1's mask), substitute the standard `Spin` → `BackUp` recovery subtree with a single slow, straight `DriveOnHeading` retreat along the entry heading, and gate it on `collision_monitor`'s `CollisionStop` state only (raw sonar reading) rather than the local costmap — `drive_on_heading` is already a registered `behavior_plugins` entry in this file, it just isn't necessarily wired into the current BT XML (`amr_navigation/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` — check and wire it in if it's a truly-unused sitting plugin, per this project's own past finding about `wait` needing to be present for BT validation).
- Re-examine `simulate_ahead_time` (currently `1.0`, raised from `0.4`) with this in mind — a longer ahead-check window means recoveries check further into the future, which is worth confirming empirically (log `ros2 param get` values live during a captured deadlock) has not made self-rejection *more* likely rather than less, since the project's comment justifying the raise ("longer collision-ahead check window") optimizes for a different goal (tolerating TF latency) than the self-rejection problem it sits next to.

### 4.5 `collision_monitor` hardening — Decision D

- Throttle `maneuver_telemetry_node`'s per-cycle console logging (visibly flooding the terminal at high frequency in the captured logs) — this is cheap and directly reduces I/O contention on whatever executor the logging shares.
- Confirm `lifecycle_manager_navigation`'s `bond_timeout` / `bond_heartbeat_period` / `attempt_respawn_reconnection` settings (see Nav2's lifecycle manager config docs) are deliberately chosen, not left at defaults by accident, and confirm what actually happens after the observed "Deactivating collision_monitor" — does the manager bring it back up automatically, or does nav2 stay down until manually relaunched? If the latter, that's a second, independent problem worth its own fix (e.g. enabling automatic respawn/reconnection) regardless of whether Decisions A–C eliminate the trigger.
- If the machine is CPU-constrained while running Webots + RViz + the full nav2 stack simultaneously (worth checking with a plain `top`/`htop` during a live deadlock), consider giving `collision_monitor` its own dedicated process (it already is one) and, if available on this build, a multi-threaded executor or higher scheduling priority, so a busy `controller_server` cannot starve it.

---

## 5. Verification / acceptance criteria

Do not consider this done until, on a fresh `sim.launch.py` + `navigation.launch.py` run:

1. A goal that only requires travel through open aisle (no doorway) produces a global path whose deviation from the local medial axis (computed the same way as in 4.1's script) stays small across the whole route — not just "looks better" in RViz.
2. A goal into AS1/AS2/AS3 completes without triggering `RECOVER_BACKUP`/`IN_PLACE_SPIN` more than once or twice transiently, and without a `Failed to make progress` abort, across at least 5 repeated attempts (not just one lucky run — this project's own status notes already flag that a single successful run isn't sufficient confirmation).
3. `collision_monitor`'s bond heartbeat never lapses for the full duration of a goal that includes a recovery event (log `/bond` or watch for the "have not received a heartbeat" message specifically during a deliberately provoked deadlock, not just during clean runs).
4. Document whatever numbers 4.1's script actually produces, and the final `(inflation_radius, cost_scaling_factor, cost_travel_multiplier)` values chosen, back into `decisions-and-gotchas.md` in this project's own established style — this session's screenshots are exactly the "full fresh end-to-end run" the project's `README.md` already flagged as unconfirmed, so whatever comes out of this work is the answer to that open item, not a new one to leave dangling.

---

## 6. References

- Nav2 Tuning Guide — "Inflation Potential Fields" section: https://docs.nav2.org/tuning/index.html (official maintainer recommendation to size inflation to create a smooth potential field across aisles/halls, not just a thin band at walls)
- `nav2_theta_star_planner` docs (alternative planner with explicit "prefer middle of spaces" weighting via `w_traversal_cost`): https://docs.ros.org/en/ros2_packages/jazzy/api/nav2_theta_star_planner
- Nav2 GitHub issue #4542 — Smac-family planner producing off-center/wobbly paths in open, low-cost regions: https://github.com/ros-navigation/navigation2/issues/4542
- Nav2 Costmap Filters / "Navigating with Keepout Zones" tutorial (preferred-lane masking for warehouse/industrial layouts — a possible escalation beyond Decision A if calibrated inflation alone proves insufficient): https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html
- Nav2 Lifecycle Manager configuration (bond timeout / heartbeat mechanics behind the `collision_monitor` crash): https://docs.nav2.org/configuration/packages/configuring-lifecycle.html
- LaValle, *Planning Algorithms*, §6.2, "maximum-clearance roadmap" / generalized Voronoi diagram: https://lavalle.pl/planning/node270.html
- Blaer, "Robot Path Planning Using Generalized Voronoi Diagrams": https://www.cs.columbia.edu/~pblaer/projects/path_planner/voronoi.html
- Wang et al., "LE-HG-PRM: A Structure-Aware Roadmap Planner for Intelligent Warehouse Logistics," *Robotics* (2026) — recent academic treatment of the same narrow-aisle/structured-warehouse planning problem, for context if a full planner replacement is ever considered: https://www.mdpi.com/2218-6581/15/7/122 (via DOI 10.3390/robotics15070122)
