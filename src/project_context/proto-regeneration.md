# PROTO Regeneration Pipeline

**`amr_simulation/webots/protos/amr_robot.proto` is a generated artifact.**
It is NOT hand-edited (except historically for one-off diagnosis) and it
is NOT rebuilt automatically by `colcon build`. If you change anything
under `amr_description/urdf/` — geometry, sensor placement, inertia, the
`<webots>` plugin block — and don't run this pipeline, Webots will keep
simulating the OLD robot until you do.

## The script

`amr_simulation/scripts/fix_proto.py` — post-processes the raw
`urdf2webots` output to fix things that tool cannot do on its own (see
the docstring at the top of that file for the full, current list of
fixes and why each exists — mesh path portability, collision origin
transforms, native sensor injection, the Lidar self-occlusion offsets,
wheel sensor renaming, invalid Physics stripping). Read that docstring,
not just this file, before changing it — it's the more detailed and more
likely to be current of the two.

## Full regeneration sequence

Run every step in the **same shell** (sourcing must happen once, not per
command, but everything after `xacro` depends on `AMENT_PREFIX_PATH`
being set for `package://` mesh resolution to work):

```bash
source /opt/ros/humble/setup.zsh
source /home/ironvolk/ros/install/setup.zsh   # after colcon build, see below

cd /home/ironvolk/ros

# 1. Rebuild amr_description first if you changed any xacro file
colcon build --packages-select amr_description
source install/setup.zsh

# 2. Flatten the xacro tree to a real URDF file, Webots variant
xacro src/amr_description/urdf/robots/amr.urdf.xacro use_webots:=true > amr_flat.urdf
check_urdf amr_flat.urdf   # sanity check before feeding it to urdf2webots

# 3. Regenerate the raw PROTO
rm -rf src/amr_simulation/webots/protos/amr_robot_textures
python3 -m urdf2webots.importer \
  --input=amr_flat.urdf \
  --output=src/amr_simulation/webots/protos/amr_robot.proto \
  --box-collision

# 4. Apply the fixups
python3 src/amr_simulation/scripts/fix_proto.py \
  src/amr_simulation/webots/protos/amr_robot.proto
# -> should print "fix_proto.py: all checks passed"

# 5. Clean up and rebuild amr_simulation so the new .proto is installed
rm -f amr_flat.urdf
colcon build --packages-select amr_simulation
source install/setup.zsh
```

## Why `check_urdf` and the assertions at the end of `fix_proto.py` matter

Both are cheap ways to catch a broken regeneration *before* burning a
launch-and-observe cycle in Webots. `fix_proto.py`'s final assertions
check brace/bracket balance and exact expected counts of injected nodes —
if urdf2webots's output format ever shifts (e.g. after a version bump),
the string-based replacements in step 4 can silently no-op instead of
erroring, and the assertions are what catch that instead of you
discovering it as "gravity stopped working again" three steps later.

## When you do NOT need to run this

Changes to `controllers.yaml`, `ekf.yaml`, `amcl.yaml`,
`mapper_params_online_async.yaml`, `simulation.wbt`'s spawn pose, or any
launch file — none of these are baked into the PROTO. They're loaded
fresh at every `ros2 launch`. Only geometry/sensor/inertia changes in the
xacro tree need this pipeline.
