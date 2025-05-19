file(GLOB_RECURSE ${PROJECT_NAME}_COMMON_SOURCES "common/*.cc")
file(GLOB_RECURSE ${PROJECT_NAME}_CHASSIS_SOURCES "chassis/*.cc")
file(GLOB_RECURSE ${PROJECT_NAME}_ARM_SOURCES "arm/*.cc")

ament_auto_add_executable(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_COMMON_SOURCES}
    ${${PROJECT_NAME}_CHASSIS_SOURCES}
    ${${PROJECT_NAME}_ARM_SOURCES}
    main.cc)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_libs)

add_subdirectory(tests)
