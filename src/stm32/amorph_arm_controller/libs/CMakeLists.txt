
set(BUILD_TESTS OFF)

add_subdirectory(rpc_core)

add_subdirectory(librm)

add_subdirectory(tinympc)

add_library(${PROJECT_NAME}_libs INTERFACE)

target_link_libraries(${PROJECT_NAME}_libs INTERFACE
        rpc_core
        rm
        tinympcstatic
)