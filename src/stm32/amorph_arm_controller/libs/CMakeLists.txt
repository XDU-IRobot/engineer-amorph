
set(BUILD_TESTS OFF)

add_subdirectory(rpc_core)

add_subdirectory(librm)

set(FASTBUILD ON)
set(CMSISCORE ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS)
add_subdirectory(CMSIS-DSP)

add_library(${PROJECT_NAME}_libs INTERFACE)

target_link_libraries(${PROJECT_NAME}_libs INTERFACE
        rpc_core
        rm
        CMSISDSP
)