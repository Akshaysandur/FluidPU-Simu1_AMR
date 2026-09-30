# Raspberry Pi environment (source, don't execute). Assumes ROS 2 Jazzy installed via apt
# (/opt/ros/jazzy) and this robot_side/ tree colcon-built at ~/amr_ws on the Pi.
export CYCLONEDDS_URI=file:///home/akshay/robot_side/cyclone_lan.xml   # path AS COPIED ONTO THE PI
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42                      # must match the dev machine's tools/env_hw.sh
source /opt/ros/jazzy/setup.bash
source ~/amr_ws/install/setup.bash 2>/dev/null
