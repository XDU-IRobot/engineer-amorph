
add_executable(arm_controller)
file(GLOB_RECURSE ARM_CONTROLLER_SOURCES
        ./src/*.c
        ./src/*.cc
        ./rpcidl/*.c
        ./rpcidl/*.cpp
)

target_sources(arm_controller PRIVATE
        ${ARM_CONTROLLER_SOURCES}
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},SOURCES>
)
target_include_directories(arm_controller PRIVATE
        ./include
        ./rpcidl
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},INCLUDE_DIRECTORIES>
)
target_link_libraries(arm_controller PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},LINK_LIBRARIES>
)