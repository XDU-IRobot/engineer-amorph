cmake_minimum_required(VERSION 3.8)
project(fluorite_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE ${PROJECT_NAME}_msgs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
file(GLOB_RECURSE ${PROJECT_NAME}_srvs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*.srv)
file(GLOB_RECURSE ${PROJECT_NAME}_actions RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*.action)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${${PROJECT_NAME}_msgs}
  ${${PROJECT_NAME}_srvs}
  ${${PROJECT_NAME}_actions}
  DEPENDENCIES std_msgs trajectory_msgs geometry_msgs
)

ament_package()
