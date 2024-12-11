cmake_minimum_required(VERSION 3.13)

find_package(Eigen3 REQUIRED)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/librm)

add_library(tinyfsm INTERFACE)
target_include_directories(tinyfsm INTERFACE ${CMAKE_CURRENT_LIST_DIR}/tinyfsm/include)

add_library(filters INTERFACE)
target_include_directories(filters INTERFACE ${CMAKE_CURRENT_LIST_DIR}/DSP-Cpp-filters/lib)

add_subdirectory(edge-trigger)

add_subdirectory(tweeny)

add_subdirectory(pfr)

add_library(${PROJECT_NAME}_libs INTERFACE)
target_link_libraries(${PROJECT_NAME}_libs INTERFACE 
        rm 
        tinyfsm 
        Eigen3::Eigen 
        filters 
        edge_trigger
        tweeny
        Boost::pfr)