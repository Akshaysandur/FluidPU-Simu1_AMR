cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config scripts behavior_trees maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/odom_covariance_wrapper.py
    scripts/sensor_wrapper_node.py
    scripts/aruco_detector.py
    scripts/visual_docker.py
    scripts/maneuver_telemetry_node.py
    scripts/calibrate_inflation.py
    scripts/fleet_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
