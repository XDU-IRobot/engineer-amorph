#pragma once

#include <cstdint>

/// MainLoop 执行时间（µs），由 MainLoop 写入，UI 定时读取
extern volatile long long g_mainloop_time_us;

namespace arm_ui {

/// 创建机械臂可视化 UI（在 LVGL 初始化之后调用）
void create();

}  // namespace arm_ui

