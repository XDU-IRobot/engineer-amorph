/// @brief
/// 定义跨子系统的复杂动作（如升降图传杆、启停小陀螺）

#pragma once

#include "shared_resources.hpp"
#include "global_objects.hpp"
#include "ui/ui.h"

inline void InitUserInput() { auto &shared = SharedResources::GetInstance(); }

inline void UiInit() {
  auto &shared = SharedResources::GetInstance();
  using RC = rm::device::RefereeProtocol<GlobalConfig::kRefereeRev>::RemoteControl;

  ui_self_id = 102;  // default to B2
  ui_init_g();
  shared.kh.OnKeyDown(RC::kZ, [&](const rm::modules::KeyEvent &e) {  // 按Z刷新ui
    UNUSED(e);
    ui_init_g();
  });
}