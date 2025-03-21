cmake_minimum_required(VERSION 3.8)
project(fluorite_custom_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS true)

add_subdirectory(libs)

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/*.cc)
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

target_link_libraries(${PROJECT_NAME} PRIVATE fluorite)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})