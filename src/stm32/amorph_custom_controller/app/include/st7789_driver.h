#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

/**
 * @brief Initialize the ST7789 display via LVGL's built-in driver.
 *        Sets up SPI4 DMA transfer, GPIO for CS/DC/RST/BL,
 *        and creates the LVGL display object.
 * @return pointer to the created lv_display_t, or NULL on failure
 */
lv_display_t *st7789_driver_init(void);

#ifdef __cplusplus
}
#endif

