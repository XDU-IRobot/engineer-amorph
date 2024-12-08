cmake_minimum_required(VERSION 3.8)
project(fluorite_arm)

set(CMAKE_CXX_STANDARD 20)
add_definitions(-std=c++20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cc")
ament_auto_add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
if (CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_options(${PROJECT_NAME} PRIVATE -O3)
  target_compile_options(${PROJECT_NAME} PRIVATE -DNDEBUG)
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  target_compile_options(${PROJECT_NAME} PRIVATE -O3 -g)
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(${PROJECT_NAME} PRIVATE -O0 -g)
endif()

# add_subdirectory(libs)
# target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_libs)
# target_include_directories(${PROJECT_NAME} PRIVATE include)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY launch
#         DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config
#         DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
