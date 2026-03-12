
set(BUILD_TESTS OFF)

add_subdirectory(librm)

set(FASTBUILD ON)
set(CMSISCORE ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS)
add_subdirectory(CMSIS-DSP)

set(LV_BUILD_CONF_PATH ${CMAKE_CURRENT_LIST_DIR}/lv_conf.h)
set(CONFIG_LV_BUILD_DEMOS OFF)
set(CONFIG_LV_BUILD_EXAMPLES OFF)
add_subdirectory(lvgl)
target_link_libraries(lvgl PRIVATE stm32cubemx)     # 启用了dma2d，lvgl需要调用hal库

add_library(${PROJECT_NAME}_libs INTERFACE)

target_link_libraries(${PROJECT_NAME}_libs INTERFACE
        rm
        CMSISDSP
        lvgl::lvgl
)