
#pragma once

#include "spi.h"

/**
 * @brief   基于SPI驱动的WS2812 LED控制类
 */
class WS2812Led {
  static constexpr uint8_t kWS2812LowLevel = 0xc0;
  static constexpr uint8_t kWS2812HighLevel = 0xf0;

 public:
  explicit WS2812Led(SPI_HandleTypeDef &hspi) : hspi_(&hspi) {}

  void SetColor(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++) {
      txbuf[7 - i] = (((g >> i) & 0x01) ? kWS2812HighLevel : kWS2812LowLevel) >> 1;
      txbuf[15 - i] = (((r >> i) & 0x01) ? kWS2812HighLevel : kWS2812LowLevel) >> 1;
      txbuf[23 - i] = (((b >> i) & 0x01) ? kWS2812HighLevel : kWS2812LowLevel) >> 1;
    }
    HAL_SPI_Transmit(hspi_, &res, 0, 0xffff);
    while (hspi_->State != HAL_SPI_STATE_READY) {
      // wait for SPI to be ready
    }
    HAL_SPI_Transmit(hspi_, txbuf, 24, 0xffff);
    for (int i = 0; i < 100; i++) {
      HAL_SPI_Transmit(hspi_, &res, 1, 0xffff);
    }
  }

 private:
  SPI_HandleTypeDef *hspi_;
};

namespace led_pattern {
/**
 * @brief 黄灯闪烁
 * @note  闪烁周期为1000ms（500ms开，500ms关）
 */
class YellowFlash : public rm::modules::RgbLedPattern {
 public:
  YellowFlash() = default;

  Rgb Update(TimePoint now) override {
    auto elapsed = ElapsedMs(start_time_, now);
    // 闪烁周期：1000ms
    uint32_t phase = elapsed % 1000;
    if (phase < 500) {
      return Rgb(255, 255, 0);
    }
    return Rgb(0, 0, 0);
  }

  void Reset(TimePoint now) override { start_time_ = now; }

 private:
  TimePoint start_time_;
};
}  // namespace led_pattern