
########## left
add_executable(left_arm)
file(GLOB_RECURSE ARM_CONTROLLER_SOURCES
        ./src/*.c
        ./src/*.cc
)

target_sources(left_arm PRIVATE
        ${ARM_CONTROLLER_SOURCES}
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},SOURCES>
)
target_include_directories(left_arm PRIVATE
        ./include
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},INCLUDE_DIRECTORIES>
)
target_link_libraries(left_arm PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},LINK_LIBRARIES>
)
target_compile_definitions(left_arm PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},COMPILE_DEFINITIONS>
        I_AM_LEFT_ARM
)

########## right
add_executable(right_arm)
file(GLOB_RECURSE ARM_CONTROLLER_SOURCES
        ./src/*.c
        ./src/*.cc
)

target_sources(right_arm PRIVATE
        ${ARM_CONTROLLER_SOURCES}
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},SOURCES>
)
target_include_directories(right_arm PRIVATE
        ./include
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},INCLUDE_DIRECTORIES>
)
target_link_libraries(right_arm PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},LINK_LIBRARIES>
)
target_compile_definitions(right_arm PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},COMPILE_DEFINITIONS>
        I_AM_RIGHT_ARM
)