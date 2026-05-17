// /**
//  * @file st7789_driver.c
//  * @brief ST7789 display driver for LVGL using STM32 HAL SPI4 with DMA.
//  *
//  * Pin mapping (from CubeMX .ioc):
//  *   SPI4_SCK  -> PE12
//  *   SPI4_MOSI -> PE6
//  *   SPI4_MISO -> PE5 (unused for display)
//  *   TFT_CS    -> PC4  (active low)
//  *   TFT_DC    -> PC3  (0=cmd, 1=data)
//  *   TFT_RST   -> PC2  (active low)
//  *   TFT_BL    -> PC5  (backlight, active high)
//  */
//
// #include <string.h>
//
// #include <lvgl/src/draw/dma2d/lv_draw_dma2d.h>
//
// #include "st7789_driver.h"
// #include "spi.h"
// #include "main.h"
// #include "dma2d.h"
//
// /* ── Display parameters ─────────────────────────────────────────────── */
// #define LCD_H_RES 240
// #define LCD_V_RES 320
//
// /* Use partial rendering to save RAM.
//  * Buffer = 10 lines * 240 pixels * 2 bytes/pixel = 4800 bytes per buffer.
//  * Double-buffered so LVGL can render into one while DMA sends the other. */
// #define LCD_BUF_LINES 20
// static uint8_t buf1[LCD_H_RES * LCD_BUF_LINES * 2];
// static uint8_t buf2[LCD_H_RES * LCD_BUF_LINES * 2];
//
// /* Keep a reference so the DMA-complete callback can call lv_display_flush_ready */
// static lv_display_t *s_disp = NULL;
//
// /* Flag: is a DMA transfer in progress? */
// static volatile bool s_dma_busy = false;
//
// /* ── Low-level GPIO helpers ─────────────────────────────────────────── */
// static inline void lcd_cs_low(void) {
//   // HAL_GPIO_WritePin(TFT_CS_GPIO_Port,  TFT_CS_Pin,  GPIO_PIN_RESET);
// }
// static inline void lcd_cs_high(void) {
//   // HAL_GPIO_WritePin(TFT_CS_GPIO_Port,  TFT_CS_Pin,  GPIO_PIN_SET);
// }
// static inline void lcd_dc_cmd(void) { HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET); }
// static inline void lcd_dc_data(void) { HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET); }
// static inline void lcd_rst_low(void) { HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET); }
// static inline void lcd_rst_high(void) { HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET); }
// static inline void lcd_bl_on(void) { HAL_GPIO_WritePin(TFT_BL_GPIO_Port, TFT_BL_Pin, GPIO_PIN_SET); }
// static inline void lcd_bl_off(void) { HAL_GPIO_WritePin(TFT_BL_GPIO_Port, TFT_BL_Pin, GPIO_PIN_RESET); }
//
// /* ── LVGL send_cmd callback (blocking / polling) ────────────────────── */
// static void st7789_send_cmd(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, const uint8_t *param,
//                             size_t param_size) {
//   (void)disp;
//
//   /* Wait for any pending DMA transfer to finish */
//   while (s_dma_busy) {
//   }
//
//   lcd_cs_low();
//
//   /* Send command byte(s) */
//   lcd_dc_cmd();
//   HAL_SPI_Transmit(&hspi4, (uint8_t *)cmd, cmd_size, HAL_MAX_DELAY);
//
//   /* Send parameter byte(s) if any */
//   if (param_size > 0) {
//     lcd_dc_data();
//     HAL_SPI_Transmit(&hspi4, (uint8_t *)param, param_size, HAL_MAX_DELAY);
//   }
//
//   lcd_cs_high();
// }
//
// /* ── LVGL send_color callback (DMA transfer) ────────────────────────── */
// static void st7789_send_color(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, uint8_t *param,
//                               size_t param_size) {
//   (void)disp;
//
//   /* Wait for any pending DMA transfer to finish */
//   while (s_dma_busy) {
//   }
//
//   lcd_cs_low();
//
//   /* Send command (set column / page address, write memory start, etc.) */
//   lcd_dc_cmd();
//   HAL_SPI_Transmit(&hspi4, (uint8_t *)cmd, cmd_size, HAL_MAX_DELAY);
//
//   /* Switch to data mode and start DMA for pixel data */
//   lcd_dc_data();
//   s_dma_busy = true;
//   HAL_SPI_Transmit_DMA(&hspi4, param, param_size);
//   /* lv_display_flush_ready() will be called from HAL_SPI_TxCpltCallback */
// }
//
// /* ── DMA transfer complete callback (called from IRQ context) ────────── */
// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
//   if (hspi->Instance == SPI4) {
//     lcd_cs_high();
//     s_dma_busy = false;
//     if (s_disp) {
//       lv_display_flush_ready(s_disp);
//     }
//   }
// }
//
// void DMA2D_TransferCompleteCallback(DMA2D_HandleTypeDef *hdma2d) {
//   (void)hdma2d;
//   lv_draw_dma2d_transfer_complete_interrupt_handler();
// }
//
// /* ── Hardware reset sequence ────────────────────────────────────────── */
// static void st7789_hw_reset(void) {
//   lcd_rst_high();
//   HAL_Delay(10);
//   lcd_rst_low();
//   HAL_Delay(10);
//   lcd_rst_high();
//   HAL_Delay(120);
// }
//
// /* ── Public: initialise ST7789 + LVGL display ───────────────────────── */
// lv_display_t *st7789_driver_init(void) {
//   HAL_DMA2D_RegisterCallback(&hdma2d, HAL_DMA2D_TRANSFERCOMPLETE_CB_ID, DMA2D_TransferCompleteCallback);
//
//   /* 1. Hardware reset */
//   st7789_hw_reset();
//
//   /* 2. Turn on backlight */
//   lcd_bl_on();
//
//   /* 3. Create the LVGL display using the built-in ST7789 driver.
//    *    The driver will send the standard MIPI DCS init sequence
//    *    (sleep out, pixel format, display on, etc.) via our callbacks. */
//   s_disp = lv_st7789_create(LCD_H_RES, LCD_V_RES, LV_LCD_FLAG_NONE, st7789_send_cmd, st7789_send_color);
//
//   if (s_disp == NULL) return NULL;
//
//   /* 4. Fix byte order: STM32 is little-endian but ST7789 expects big-endian
//    *    RGB565 pixel data. Use SWAPPED format so LVGL renders with bytes
//    *    already in the order the controller expects (MSB first). */
//   lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
//
//   /* 5. Assign partial-mode double buffers */
//   lv_display_set_buffers(s_disp, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
//
//   /* 6. Rotate to landscape (320x240) */
//   lv_display_set_rotation(s_disp, LV_DISPLAY_ROTATION_270);
//
//   return s_disp;
// }
