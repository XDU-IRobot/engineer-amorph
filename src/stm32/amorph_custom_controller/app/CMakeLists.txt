
add_executable(app_main)
file(GLOB_RECURSE APP_SOURCES
        ./src/*.c
        ./src/*.cc
)
target_sources(app_main PRIVATE
        ${APP_SOURCES}
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},SOURCES>
)
target_include_directories(app_main PRIVATE
        ./include
        ./src/garbage
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},INCLUDE_DIRECTORIES>
)
target_link_libraries(app_main PRIVATE
        $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},LINK_LIBRARIES>
)