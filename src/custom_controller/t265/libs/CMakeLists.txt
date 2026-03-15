
find_package(Boost REQUIRED COMPONENTS system)  # boost::asio

find_package(spdlog REQUIRED)

add_library(${PROJECT_NAME}_third_party INTERFACE)

add_subdirectory(librm)

target_link_libraries(${PROJECT_NAME}_third_party INTERFACE
    realsense2
    Boost::system
    spdlog::spdlog
    rm
)
