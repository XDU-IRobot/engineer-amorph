#pragma once

#include "adc.h"

class VbusAdc {
 public:
  explicit VbusAdc(ADC_HandleTypeDef &hadc = hadc1) : hadc_(&hadc) {}

  void Begin() {
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, &raw_adc_val_, 1);
  }

  float vbus() const { return (raw_adc_val_ * 3.3f / 65535) * 11.0f; }

 private:
  ADC_HandleTypeDef *hadc_{};
  uint32_t raw_adc_val_{};
};