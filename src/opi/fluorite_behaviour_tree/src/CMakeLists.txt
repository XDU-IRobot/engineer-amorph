
file(GLOB_RECURSE NODES_SOURCES nodes/*.cc)

ament_auto_add_library(${PROJECT_NAME}_bt_nodes STATIC ${NODES_SOURCES})
ament_auto_add_executable(model_exporter model_exporter.cc)
ament_auto_add_executable(bt_executor bt_executor.cc)

target_link_libraries(${PROJECT_NAME}_bt_nodes ${PROJECT_NAME}_libs)
target_link_libraries(model_exporter ${PROJECT_NAME}_bt_nodes)
target_link_libraries(bt_executor ${PROJECT_NAME}_bt_nodes ${PROJECT_NAME}_libs)
