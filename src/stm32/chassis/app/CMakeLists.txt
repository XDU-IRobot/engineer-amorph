
add_executable(app_main)
file(GLOB_RECURSE ARM_CONTROLLER_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/*.c
        ${CMAKE_CURRENT_LIST_DIR}/*.cc
)

target_sources(app_main PRIVATE
        ${ARM_CONTROLLER_SOURCES}
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},SOURCES>
)
target_include_directories(app_main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},INCLUDE_DIRECTORIES>
)
target_link_libraries(app_main PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},LINK_LIBRARIES>
)

target_link_options(app_main PRIVATE -u _printf_float)