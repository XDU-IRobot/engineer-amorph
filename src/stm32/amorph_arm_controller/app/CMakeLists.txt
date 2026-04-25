file(GLOB_RECURSE SOURCES ./src/*.c ./src/*.cc)

add_executable(app_main)
target_sources(app_main PRIVATE
        ${SOURCES}
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},SOURCES>
)
target_include_directories(app_main PRIVATE
        ./include
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},INCLUDE_DIRECTORIES>
)
target_link_libraries(app_main PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},LINK_LIBRARIES>
)
target_compile_definitions(app_main PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},COMPILE_DEFINITIONS>
)
