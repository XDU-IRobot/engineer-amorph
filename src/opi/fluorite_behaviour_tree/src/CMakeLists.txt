
file(GLOB_RECURSE NODES_SOURCES nodes/*.cc)

ament_auto_add_library(${PROJECT_NAME}_bt_nodes STATIC ${NODES_SOURCES})
ament_auto_add_executable(model_exporter model_exporter.cc)
ament_auto_add_executable(bt_coordinator bt_coordinator.cc)

target_link_libraries(model_exporter ${PROJECT_NAME}_bt_nodes)
target_link_libraries(bt_coordinator ${PROJECT_NAME}_bt_nodes)
