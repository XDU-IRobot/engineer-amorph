cmake_minimum_required(VERSION 3.20)
project(amorph_interfaces)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/SystemStatus.msg

  DEPENDENCIES std_msgs
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
