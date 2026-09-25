#!/usr/bin/env python3
"""Post-process amr_robot.proto after every urdf2webots regeneration.

WHY THIS EXISTS: urdf2webots cannot see <webots>/<gazebo> xacro extensions,
and (in our specific setup) needs help placing the Lidar so it doesn't scan
into its own body. This script is the second half of the regeneration
pipeline. The command that produces the input .proto (run from
amr_simulation/webots/protos/, with amr_description already colcon-built so
package:// URIs resolve):

  xacro <install>/amr_description/share/amr_description/urdf/robots/amr.urdf.xacro \\
      use_webots:=true > amr_flat.urdf
  python3 -m urdf2webots.importer --input=amr_flat.urdf \\
      --output=amr_robot.proto --relative-path-prefix="meshes/"

IMPORTANT: do NOT pass --robot-name alongside --output=*.proto. Passing both
flips urdf2webots into "generate a Robot node string" mode instead of
"generate a PROTO file" (see its own --robot-name help text) - it silently
writes nothing to outputFile at all (exit code 0, no error) because the
whole `if isProto:` file-writing branch in importer.py is skipped. The
robot's internal name is derived automatically from the --output filename's
basename ("amr_robot" from "amr_robot.proto") when --robot-name is omitted,
so nothing is lost by leaving it out.

2WD + caster transition note (github.com/ravithakur-projects/
ROS-Three-Wheeled-Robot-Navigation-Project geometry): this urdf2webots
invocation, unlike whatever produced the original mecanum-era proto,
already emits correctly Pose-wrapped boundingObjects for both the chassis
Box and the wheel Cylinders (translation/rotation preserved) - verified by
inspecting a fresh regeneration output directly. The old "primitive
boundingObjects silently drop <collision><origin>" workaround this script
used to apply is therefore no longer needed and has been removed rather
than carried forward with stale (pre-2WD) chassis/wheel dimensions.

Structural note: this targets the base_footprint-bypass URDF (base_link is
the true root under use_webots:=true - see base.urdf.xacro). Because of
that, urdf2webots puts boundingObject/physics directly on the Robot node
itself (no wrapping "base_link" Solid exists - Robot's own `name` field is
bound via `name IS name` to the exposed PROTO field instead), and the
sensor/caster sub-solids (laser_frame/imu_link/cam_1_link/caster_wheel_link)
sit one indentation level shallower than they would if base_footprint were
present.

Fixes applied, in order:
  1. Mesh URLs come out as absolute paths tied to this machine's install/
     directory (--relative-path-prefix did not actually change this, verified
     empirically) -> rewritten relative to this .proto file's own location,
     pointing at the mesh copies kept alongside it in protos/meshes/.
  2. Injects native Webots sensor nodes (Lidar, IMU triplet, Camera+
     RangeFinder) that urdf2webots does not create from <gazebo>/<webots>
     xacro extensions - it does not parse those at all.
       - Lidar `type` is set to "rotating" (not the default "fixed") for a
         genuine 360-degree fieldOfView, matching the real RPLidar S2
         hardware and Webots' own official Lidar sample (lidar.wbt), which
         also uses type "rotating".
       - Lidar gets its own small `translation 0 0 0.03` INSIDE the Lidar
         node (independent of the visual mesh, which is a sibling in the
         same Solid): the lidar.stl visualization mesh's bounding box
         straddles the origin in all 3 axes, i.e. it fully encloses its
         own local origin. Without this offset, every one of the sensor's
         360 rays hits the inside of its own visualization dome within
         millimeters and returns .inf in every direction, regardless of
         how high the parent Solid is mounted (verified by raising the
         mount height alone first, which did NOT fix it - the dome moves
         up with the sensor).
  3. Renames urdf2webots's auto-generated wheel PositionSensors from
     "<joint>_sensor" to the "<side>_wheel_sensor" naming used in
     sim_control.urdf.xacro's device/plugin config.
  4. Injects contactMaterial into the wheel and caster Solid blocks
     (urdf2webots never emits this field - it doesn't parse <gazebo>
     friction tags). Both wheels share one "wheel_material" (plain
     differential wheels, no diagonal roller split like the old mecanum
     macro needed); the caster gets its own "caster_material" tuned to
     near-zero friction in simulation.wbt's ContactProperties.
  5. Strips the invalid Physics node urdf2webots puts on every plain
     (non-jointed) child Solid - laser_frame, imu_link, cam_1_link, and now
     caster_wheel_link. Webots only allows a Physics node on the Robot's
     direct base Solid/fields or on a Joint's endPoint Solid - these four
     are plain fixed children, so an independent Physics node on them is
     invalid and breaks/disables proper physics for the body (this was one
     root cause of "gravity does nothing" - see
     project_context/decisions-and-gotchas.md).

Also note: the chassis mount height for the Lidar's PARENT Solid (set in
amr.urdf.xacro's <xacro:lidar xyz_offset=.../> call, currently 0.22) is a
SEPARATE fix from #2 above - that one clears the chassis roof; this
script's own +0.03 offset clears the Lidar's OWN visualization mesh. Both
were needed; neither alone was sufficient.
"""
import re
import sys

PROTO_PATH = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/ironvolk/ros/src/amr_simulation/webots/protos/amr_robot.proto"

with open(PROTO_PATH) as f:
    content = f.read()

# 1. Portable mesh paths
content = re.sub(
    r'/home/ironvolk/ros/install/amr_description/share/amr_description/meshes/',
    'meshes/',
    content,
)

# 2. Native sensor injection (indentation matches base_link-as-root: one
#    level shallower than it would be with a wrapping base_link Solid)
content = content.replace(
    '        ]\n        name "laser_frame"',
    '          Lidar {\n'
    '            translation 0 0 0.03\n'
    '            name "Lidar"\n'
    '            type "rotating"\n'
    '            horizontalResolution 360\n'
    '            numberOfLayers 1\n'
    '            fieldOfView 6.28318\n'
    '            minRange 0.05\n'
    '            maxRange 12.0\n'
    '          }\n'
    '        ]\n        name "laser_frame"',
)

content = re.sub(
    r'(\s*)\}\n(\s*)\]\n(\s*)name "imu_link"',
    r'\1}\n'
    r'\2  InertialUnit {\n\2    name "IMU"\n\2  }\n'
    r'\2  Accelerometer {\n\2    name "accelerometer"\n\2  }\n'
    r'\2  Gyro {\n\2    name "gyroscope"\n\2  }\n'
    r'\2]\n\3name "imu_link"',
    content,
)

content = content.replace(
    '        ]\n        name "cam_1_link"',
    '          Camera {\n'
    '            name "camera"\n'
    '            width 424\n'
    '            height 240\n'
    '            fieldOfView 1.5184\n'
    '          }\n'
    '          RangeFinder {\n'
    '            name "range_finder"\n'
    '            width 424\n'
    '            height 240\n'
    '            fieldOfView 1.5184\n'
    '            minRange 0.05\n'
    '            maxRange 1.5\n'
    '          }\n'
    '        ]\n        name "cam_1_link"',
)

# 3. Wheel PositionSensor renames
for side in ['left', 'right']:
    content = content.replace(
        f'name "{side}_wheel_joint_sensor"',
        f'name "{side}_wheel_sensor"',
    )

# 4. Inject contactMaterial into wheel and caster Solid blocks.
for link_name, material in [
    ('left_wheel_link',  'wheel_material'),
    ('right_wheel_link', 'wheel_material'),
]:
    pattern = re.compile(
        r'(          name "' + re.escape(link_name) + r'"\n'
        r'(          boundingObject Pose \{.*?\}\n))'
        r'(          physics)',
        re.DOTALL
    )
    replacement = (
        r'\1'
        r'          contactMaterial "' + material + r'"\n'
        r'\3'
    )
    new_content, n = pattern.subn(replacement, content)
    assert n == 1, \
        f"contactMaterial injection: expected 1 match for '{link_name}', got {n}"
    content = new_content

# Caster's boundingObject is a Pose-wrapped Cylinder (matches the real
# caster_1 mesh's disc footprint - see base.urdf.xacro) at one indentation
# level shallower than the wheels (a direct Robot child, not nested inside
# a HingeJoint's endPoint Solid).
caster_pattern = re.compile(
    r'(        name "caster_wheel_link"\n'
    r'(        boundingObject Pose \{.*?\}\n))'
    r'(        physics)',
    re.DOTALL
)
content, n = caster_pattern.subn(
    r'\1        contactMaterial "caster_material"\n\3', content
)
assert n == 1, f"contactMaterial injection: expected 1 match for 'caster_wheel_link', got {n}"

# 5. Strip invalid Physics from fixed (non-jointed) sensor/caster sub-solids
for frame_name in ['laser_frame', 'imu_link', 'cam_1_link', 'caster_wheel_link']:
    pattern = (
        r'(name "' + frame_name + r'"\n)'
        r'((?:.*\n)*?)'
        r'(\s*)physics Physics \{[^}]*\}\n'
    )
    new_content, n = re.subn(pattern, r'\1\2', content, count=1)
    assert n == 1, f"expected to strip exactly one Physics block for {frame_name}, stripped {n}"
    content = new_content

with open(PROTO_PATH, 'w') as f:
    f.write(content)

# Sanity checks
assert content.count('{') == content.count('}'), "brace mismatch after fixup!"
assert content.count('[') == content.count(']'), "bracket mismatch after fixup!"
for name in ['Lidar {', 'InertialUnit {', 'Accelerometer {', 'Gyro {',
             'Camera {', 'RangeFinder {']:
    assert content.count(name) == 1, f"expected exactly one {name}, found {content.count(name)}"
assert 'type "rotating"' in content, "Lidar type=rotating fix did not land!"
assert content.count('boundingObject Pose {') == 4, \
    f"expected 4 Pose-wrapped boundingObjects (1 base + 2 wheels + 1 caster), found {content.count('boundingObject Pose {')}"
assert 'install/amr_description' not in content, "absolute mesh path leaked through!"
assert content.count('physics Physics {') == 3, \
    f"expected exactly 3 Physics nodes (Robot base + 2 wheels), found {content.count('physics Physics {')}"
wheel_count = content.count('contactMaterial "wheel_material"')
caster_count = content.count('contactMaterial "caster_material"')
assert wheel_count == 2, \
    f"expected 2 wheel_material contactMaterial fields, found {wheel_count}"
assert caster_count == 1, \
    f"expected 1 caster_material contactMaterial field, found {caster_count}"

print("fix_proto.py: all checks passed")
