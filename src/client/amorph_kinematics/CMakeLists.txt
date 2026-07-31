cmake_minimum_required(VERSION 3.8)
project(amorph_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Eigen3 is not a standard ament package, find it manually
find_package(Eigen3 REQUIRED)

# Headers-only library
add_library(${PROJECT_NAME}_headers INTERFACE)
target_include_directories(${PROJECT_NAME}_headers INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME}_headers INTERFACE Eigen3::Eigen)

# FK node
ament_auto_add_executable(fk_node src/fk_node.cpp)
target_link_libraries(fk_node ${PROJECT_NAME}_headers)

# IK node
ament_auto_add_executable(ik_node src/ik_node.cpp)
target_link_libraries(ik_node ${PROJECT_NAME}_headers)

# Test FK-IK verification node
ament_auto_add_executable(test_fk_ik_node src/test_fk_ik_node.cpp)
target_link_libraries(test_fk_ik_node ${PROJECT_NAME}_headers)

# Install
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_headers EXPORT ${PROJECT_NAME}Targets)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_auto_package(INSTALL_TO_SHARE)

