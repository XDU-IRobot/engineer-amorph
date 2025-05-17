
file(GLOB_RECURSE GENERATED_UI_SOURCE ./*.c)
add_library(${PROJECT_NAME}_generated_ui STATIC ${GENERATED_UI_SOURCE})
target_include_directories(${PROJECT_NAME}_generated_ui PUBLIC ${CMAKE_CURRENT_LIST_DIR})
