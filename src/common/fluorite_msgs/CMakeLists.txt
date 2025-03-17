cmake_minimum_required(VERSION 3.8)
project(fluorite_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

list(APPEND ${PROJECT_NAME}_msgs
  "msg/ArmState.msg"
  "msg/ChassisState.msg"
  "msg/Pid.msg"
)
list(APPEND ${PROJECT_NAME}_srvs
  "srv/CommandChassis.srv"
  "srv/CommandSucker.srv"
  "srv/CommandStorageSystem.srv"
)
list(APPEND ${PROJECT_NAME}_actions
  "action/ArmFollowTrajectory.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${${PROJECT_NAME}_msgs}
  ${${PROJECT_NAME}_srvs}
  ${${PROJECT_NAME}_actions}
  DEPENDENCIES std_msgs trajectory_msgs geometry_msgs
)

ament_package()
