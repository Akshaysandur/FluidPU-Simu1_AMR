# Hardware mirror: real AMR follows the Webots sim

Goal: when the Webots simulation runs `amr1`'s mission, the physical robot's wheels replay the same
commands live. The simulated robot stays the "brain" (Nav2 plans and drives against the simulated map);
the real robot has no localization of its own in this mode - it only makes sense if the physical room's
walls/stations line up with the sim layout and the robot is started at the matching pose.

## What's here

- `amr_hardware/` - a ROS 2 package meant to be copied to, and built ON, the Raspberry Pi (it targets
  ROS 2 Jazzy / Ubuntu 24.04 aarch64, not this workspace's ROS 2 Humble). Nodes:
  - `dynamixel_drive` - subscribes to `/cmd_vel` (the exact topic that drives the simulated robot's wheels -
    see the docstring in `amr_hardware/dynamixel_drive.py`) and writes matching velocity commands to the
    two MX-106R motors over the U2D2.
  - `imu_node` - BNO085 fused orientation -> `/imu`. Not used for navigation in this mode; telemetry only.
  - `ultrasonic_node` - HC-SR04 -> `/ultrasonic_front`, and publishes `/hw_estop` (true) if something is
    within 0.15 m of the real robot. **This is the only thing that protects the physical robot from
    obstacles the simulated Nav2 doesn't know about** - the sim has no visibility into the real room.
- `cyclone_lan.xml`, `env.sh` - network config for the Pi side.
- `udev/99-amr-hardware.rules` - stable `/dev/dynamixel_u2d2` symlink.

On the dev machine: `tools/cyclone_lan.xml` + `tools/env_hw.sh` (LAN discovery instead of the loopback-only
profile `tools/env.sh` uses - that one is untouched, sim-only runs are unaffected).

## One-time setup

**On the Pi:**
1. `sudo apt install ros-jazzy-desktop ros-jazzy-sllidar-ros2 python3-pip`
2. `pip install dynamixel-sdk adafruit-circuitpython-bno08x gpiozero`
3. Copy `robot_side/` from this repo to the Pi (e.g. `scp -r robot_side/ pi@<PI_IP>:~/`), then on the Pi:
   ```
   sudo cp ~/robot_side/amr_hardware/udev/99-amr-hardware.rules /etc/udev/rules.d/
   sudo udevadm control --reload && sudo udevadm trigger
   mkdir -p ~/amr_ws/src && cp -r ~/robot_side/amr_hardware ~/amr_ws/src/
   cd ~/amr_ws && source /opt/ros/jazzy/setup.bash && colcon build
   ```
4. Edit `~/robot_side/cyclone_lan.xml`: set `NetworkInterfaceAddress` to the Pi's real WiFi interface name
   (`ip -4 addr` on the Pi) and `<Peer address=...>` to the dev machine's IP (`hostname -I` there).
5. **Measure the real robot and fill in `amr_hardware/config/hardware.yaml`**: `wheel_radius` and
   `wheel_separation`. The placeholders (0.033 m / 0.16 m) are the simulated TurtleBot3 Burger's, almost
   certainly wrong for this robot - until corrected, `/cmd_vel` will be driven at the wrong real speed.

**On the dev machine:**
1. Edit `tools/cyclone_lan.xml`: `NetworkInterfaceAddress` should already say `wlp2s0` (confirmed via
   `ip -4 addr` on this machine); set `<Peer address=...>` to the Pi's IP.
2. Confirm both machines can see each other: on the Pi, `source ~/robot_side/env.sh && ros2 topic list`;
   on the dev machine, `source tools/env_hw.sh && ros2 topic list` in a separate terminal - each should be
   able to `ros2 topic list` and see topics the other side will later publish (test with `ros2 topic pub
   /test std_msgs/msg/String "{data: hi}"` on one side, `ros2 topic echo /test` on the other, before wiring
   up the real robot).

## Every run

**Terminal 1-4 (dev machine, single-robot sim + nav, as usual)** - use `tools/env_hw.sh` instead of
`tools/env.sh` in each so `/cmd_vel` reaches the LAN:
```
source /home/akshay/rosws/tools/env_hw.sh
ros2 launch amr_simulation sim.launch.py
```
```
source /home/akshay/rosws/tools/env_hw.sh
ros2 launch amr_navigation navigation.launch.py
```
```
source /home/akshay/rosws/tools/env_hw.sh
ros2 launch amr_navigation fleet_control.launch.py
```
```
source /home/akshay/rosws/tools/env_hw.sh
ros2 service call /fms/full_mission std_srvs/srv/Trigger
```

**Terminal 5 (on the Pi, started any time before the mission trigger):**
```
source ~/robot_side/env.sh
ros2 launch amr_hardware hardware_mirror.launch.py
```

Stop the real robot at any time with Ctrl-C on terminal 5, or physically with the main power switch - the
software watchdog (`cmd_vel_timeout` in `hardware.yaml`) also stops the wheels if the link to the dev
machine drops.

## Known gaps / what to verify before trusting this near people or the real Station hardware

- **Not yet run on real hardware** - built and reviewed here, but I have no way to test it from this
  environment. Bench-test with the robot's wheels off the ground first (wrong sign/scale on
  `wheel_radius`/`wheel_separation` will make it spin instead of drive straight, or drive too fast/slow).
- **`XL 415` buck converter swap** - noted, doesn't change anything in this package (it only cares about
  5V/11.1V being present, not how they're regulated).
- **`/hw_estop` is a hard stop, not a re-route** - the real robot stops but the simulated one (and Nav2's
  plan) keeps going; once the sim's matching /cmd_vel says something else, the real robot will jump to
  match it, possibly still into whatever tripped the sonar. Fine for a first demo in a clear room, not
  fine as the only safety layer once you're running this near people.
- **Odometry (`/odom`) is published but unused** - a first step towards option 2 from our discussion
  (hardware running its own independent Nav2 stack), not wired into anything yet.
