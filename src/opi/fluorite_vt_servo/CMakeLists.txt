cmake_minimum_required(VERSION 3.8)
project(fluorite_vt_servo)

set(CMAKE_CXX_STANDARD 20)
add_definitions(-std=c++20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(backward_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
  add_compile_options(-DNDEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_compile_options(-O3 -g)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O0 -g)
endif()

include_directories(include)
ament_auto_add_executable(${PROJECT_NAME} src/node.cc)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_package(Eigen3 REQUIRED)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
if(${WIRINGPI_LIBRARIES} STREQUAL "WIRINGPI_LIBRARIES-NOTFOUND")
  message(WARNING "wiringPi library not found. Using mock implementation.")
else()
  target_link_libraries(${PROJECT_NAME} ${WIRINGPI_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
