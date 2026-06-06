
add_executable(cc_left)
file(GLOB_RECURSE APP_SOURCES
        ./src/*.c
        ./src/*.cc
)
target_sources(cc_left PRIVATE
        ${APP_SOURCES}
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},SOURCES>
)
target_include_directories(cc_left PRIVATE
        ./include
        ./src/garbage
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},INCLUDE_DIRECTORIES>
)
target_link_libraries(cc_left PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},LINK_LIBRARIES>
)
target_link_options(cc_left PRIVATE
        -u _printf_float
)
target_compile_definitions(cc_left PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},COMPILE_DEFINITIONS>
        I_AM_LEFT_CC
)


add_executable(cc_right)
file(GLOB_RECURSE APP_SOURCES
        ./src/*.c
        ./src/*.cc
)
target_sources(cc_right PRIVATE
        ${APP_SOURCES}
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},SOURCES>
)
target_include_directories(cc_right PRIVATE
        ./include
        ./src/garbage
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},INCLUDE_DIRECTORIES>
)
target_link_libraries(cc_right PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},LINK_LIBRARIES>
)
target_link_options(cc_right PRIVATE
        -u _printf_float
)
target_compile_definitions(cc_right PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},COMPILE_DEFINITIONS>
        I_AM_RIGHT_CC
)