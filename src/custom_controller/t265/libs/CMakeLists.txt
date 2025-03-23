
find_package(Boost REQUIRED COMPONENTS system)  # boost::asio

add_subdirectory(spdlog)

add_library(fluorite_third_party INTERFACE)

add_subdirectory(librm)

target_link_libraries(fluorite_third_party INTERFACE
    realsense2
    Boost::system
    spdlog::spdlog
    rm
)
