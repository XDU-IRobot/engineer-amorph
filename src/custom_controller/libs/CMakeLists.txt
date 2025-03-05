
find_library(log-lib log)

add_library(fluorite INTERFACE)

target_link_libraries(fluorite INTERFACE
    log
    realsense2
)
