cmake_minimum_required(VERSION 3.13)

add_subdirectory(librm)

add_library(${PROJECT_NAME}_libs INTERFACE)
target_link_libraries(${PROJECT_NAME}_libs INTERFACE
    rm
)

