cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

ament_export_dependencies(urdf)

install(
  DIRECTORY urdf launch meshes rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
