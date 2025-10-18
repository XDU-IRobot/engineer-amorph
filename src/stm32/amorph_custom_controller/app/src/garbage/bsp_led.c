#include "bsp_led.h"
#include "main.h"
extern TIM_HandleTypeDef htim5;
/**
 * @brief          aRGB show
 * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
 * @retval         none
 */
/**
 * @brief          ��ʾRGB
 * @param[in]      aRGB:0xaaRRGGBB,'aa' ��͸����,'RR'�Ǻ�ɫ,'GG'����ɫ,'BB'����ɫ
 * @retval         none
 */
void aRGB_led_show(uint32_t aRGB) {
  static uint8_t alpha;
  static uint16_t red, green, blue;
  static bool_t led_pwm_started = 0;

  if (!led_pwm_started) {
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    led_pwm_started = 1;
  }

  alpha = (aRGB & 0xFF000000) >> 24;
  red = ((aRGB & 0x00FF0000) >> 16) * alpha;
  green = ((aRGB & 0x0000FF00) >> 8) * alpha;
  blue = ((aRGB & 0x000000FF) >> 0) * alpha;

  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}
