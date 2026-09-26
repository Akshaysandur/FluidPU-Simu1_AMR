#!/bin/bash
# Stop every simulation / Nav2 / fleet process (ps-based so it never kills itself).
ps -eo pid,args | awk '/webots|nav2_|lib\/amr_navigation|ros2_supervisor|rviz2|ros2 launch|controller_manager|twist_mux|robot_state_publisher|static_transform_publisher|maneuver_telemetry|webots_ros2_driver/ && !/awk/ && !/stop_sim/ {print $1}' | xargs -r kill -9 2>/dev/null
sleep 2; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "simulation stopped"
