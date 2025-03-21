
find_package(Boost REQUIRED COMPONENTS system)  # boost::asio

add_library(fluorite INTERFACE)

add_subdirectory(librm)

target_link_libraries(fluorite INTERFACE
    realsense2
    Boost::system
    rm
)
