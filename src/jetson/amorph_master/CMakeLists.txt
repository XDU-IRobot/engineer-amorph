cmake_minimum_required(VERSION 3.8)
project(amorph_master)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_executable(pick_test_node src/pick_test_node.cc)
target_link_libraries(pick_test_node Eigen3::Eigen)

ament_auto_add_executable(visualization_test_node src/visualization_test_node.cc)
target_link_libraries(visualization_test_node Eigen3::Eigen)

ament_auto_add_executable(eu_plane_node src/eu_plane_node.cc src/rack_plane.cc)
target_link_libraries(eu_plane_node Eigen3::Eigen)

ament_auto_add_executable(single_pick_node src/single_pick_node.cc src/rack_plane.cc)
target_link_libraries(single_pick_node Eigen3::Eigen)

ament_auto_package(INSTALL_TO_SHARE config USE_SCOPED_HEADER_INSTALL_DIR)
