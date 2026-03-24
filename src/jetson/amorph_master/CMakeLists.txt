cmake_minimum_required(VERSION 3.8)
project(amorph_master)

set(CMAKE_CXX_STANDARD 20)
add_definitions(-std=c++20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(backward_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(libs)

include_directories(include/amorph_master)
include_directories(src/mtc_tasks)

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/*.cc)

generate_parameter_library(pick_place_task_params src/mtc_tasks/pick_place_task/parameters.yaml)
generate_parameter_library(pick_energy_unit_task_params src/mtc_tasks/pick_energy_unit_task/parameters.yaml)

# 创建组件库
ament_auto_add_library(${PROJECT_NAME}_component SHARED ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_libs
  pick_place_task_params
  pick_energy_unit_task_params
)

# 注册组件
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "amorph::AmorphMasterNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
