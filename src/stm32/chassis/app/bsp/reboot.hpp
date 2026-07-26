#pragma once

#include "stm32h7xx.h"

inline void RebootMCU() {
  __ASM volatile("cpsid i");  // 进入临界区，禁止所有中断
  NVIC_SystemReset();         // 重启
}