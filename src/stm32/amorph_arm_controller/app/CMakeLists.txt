file(GLOB_RECURSE SOURCES ./src/*.c ./src/*.cc)

add_executable(dual_arm)
target_sources(dual_arm PRIVATE
        ${SOURCES}
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},SOURCES>
)
target_include_directories(dual_arm PRIVATE
        ./include
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},INCLUDE_DIRECTORIES>
)
target_link_libraries(dual_arm PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},LINK_LIBRARIES>
)
target_compile_definitions(dual_arm PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},COMPILE_DEFINITIONS>
)
