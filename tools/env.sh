# Shared environment for boot_sim.sh / stop_sim.sh (source, don't execute)
export CYCLONEDDS_URI=file:///home/akshay/rosws/tools/cyclone_lo.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/ros2_humble/install/setup.bash 2>/dev/null
source /home/akshay/rosws/install/setup.bash 2>/dev/null   # AFTER the underlay: loads the patched tf2_ros
LOGS=/tmp/amr_logs; mkdir -p $LOGS
