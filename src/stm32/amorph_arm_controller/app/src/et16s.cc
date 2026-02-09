
#include "et16s.hpp"

#include "globals.hpp"
#include "fsm.hpp"

void InitUserInput() {
  // 1. 定义遥控器拨杆功能
#if defined(I_AM_LEFT_ARM)
  constexpr int kModeSwitch = rc_ch::SB;
#elif defined(I_AM_RIGHT_ARM)
  constexpr int kModeSwitch = rc_ch::SC;
#endif
  globals->rc(kModeSwitch).OnValueChange([](const int &old_value, const int &new_value) {
    // 353-up, 1024-mid, 1695-down
    if (old_value == 0) {
      return;  // 第一次触发不响应
    }
    if (new_value > 1300 && new_value < 1700) {  // 机械臂模式杆在下 - 无力
      fsm::arm.receive(fsm::event::FaultClear{});
      fsm::arm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});
    }
    if (new_value > 0 && new_value < 500) {  // 机械臂模式杆在上 - 自动模式，idle
      fsm::arm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kIdle});
    }
    if (new_value > 500 && new_value < 1100) {  // 机械臂模式杆在中 - 手动调试模式
      fsm::arm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kManual});
    }
  });
  globals->rc(rc_ch::SH).OnValueChange([](const int &old_value, const int &new_value) {
    UNUSED(old_value);
    if (new_value > 1000) {  // SH按下
      fsm::arm.receive(fsm::event::NextJoint{});
    }
  });
  globals->rc(rc_ch::SG).OnValueChange([](const int &old_value, const int &new_value) {
    UNUSED(old_value);
  });
}