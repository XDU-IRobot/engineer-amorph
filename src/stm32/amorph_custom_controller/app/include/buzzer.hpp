
#pragma once

/**
 * @brief   蜂鸣器发出指定次数的哔哔声
 * @param   beeps 哔哔声次数
 */
void BuzzerBeep(int beeps);

/**
 * @brief   异步蜂鸣器发出指定次数的哔哔声（利用TIM14）
 * @param   beeps 哔哔声次数
 */
void AsyncBuzzerBeep(int beeps);

/**
 * @brief   TIM14中断回调，供中断服务调用
 */
extern "C" void AsyncBuzzerBeep_TIM14_IRQHandler();