
find_package(Boost REQUIRED COMPONENTS system)  # boost::asio

add_library(fluorite INTERFACE)

target_link_libraries(fluorite INTERFACE
    realsense2
    Boost::system
)
