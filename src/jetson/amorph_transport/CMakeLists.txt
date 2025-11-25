cmake_minimum_required(VERSION 3.8)
project(amorph_transport)

set(CMAKE_CXX_STANDARD 20)
add_definitions(-std=c++20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(backward_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(libs)

include_directories(include/amorph_transport)
include_directories(src/mtc)

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/*.cc)

ament_auto_add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_libs
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
