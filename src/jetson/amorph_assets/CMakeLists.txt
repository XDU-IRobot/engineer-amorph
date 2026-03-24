cmake_minimum_required(VERSION 3.5)
project(amorph_assets)

find_package(ament_cmake REQUIRED)

install(DIRECTORY assets
        DESTINATION share/${PROJECT_NAME}
)

ament_package()