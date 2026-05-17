cmake_minimum_required(VERSION 3.5)
project(amorph_arm_v3_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()