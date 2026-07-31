cmake_minimum_required(VERSION 3.5)
project(amorph_arm_v5_right_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()