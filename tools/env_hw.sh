# Dev-machine environment for hardware-mirror testing (source, don't execute).
# Same as tools/env.sh but on the LAN CycloneDDS profile instead of loopback, so the dev machine's
# /cmd_vel reaches the Raspberry Pi. Use this INSTEAD of env.sh when a real robot must mirror the sim;
# use plain env.sh for sim-only runs (boot up / boot multi / boot dual) - it is untouched.
export CYCLONEDDS_URI=file:///home/akshay/rosws/tools/cyclone_lan.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42                      # must match the Pi's ROS_DOMAIN_ID (see robot_side/env.sh)
source ~/ros2_humble/install/setup.bash 2>/dev/null
source /home/akshay/rosws/install/setup.bash 2>/dev/null
LOGS=/tmp/amr_logs; mkdir -p $LOGS
