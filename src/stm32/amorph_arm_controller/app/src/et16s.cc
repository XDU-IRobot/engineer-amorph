
#include "et16s.hpp"

#include "globals.hpp"
#include "fsm.hpp"

void InitUserInput() {
  static bool force_manual_mode = false;
  // 1. 定义遥控器拨杆功能
  globals->rc(rc_ch::SD).OnValueChange([](const int &old_value, const int &new_value) {
    // 353-up, 1024-mid, 1695-down
    if (old_value == 0) {
      return;  // 第一次触发不响应
    }
    if (new_value > 1300 && new_value < 1700) {  // SD在下
      fsm::arm.receive(fsm::event::FaultClear{});
      fsm::arm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});
    }
    if (new_value > 0 && new_value < 500) {  // SD在上
      fsm::arm.receive(fsm::event::GoToPosReq{{0, 1.9f, -1.57f, 0, 0, 0}});
    }
    if (new_value > 500 && new_value < 1100) {  // SD在中
      if (force_manual_mode) {                  // 如果SF拨到外面，强制手动模式
        fsm::arm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kManual});
      } else {
        fsm::arm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kIdle});
      }
    }
  });
  globals->rc(rc_ch::SF).OnValueChange([](const int &old_value, const int &new_value) {
    UNUSED(old_value);
    if (new_value < 1000) {  // SF拨到外面
      force_manual_mode = true;
    } else {
      force_manual_mode = false;
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
    if (new_value > 1300 && new_value < 1700) {  // SG在外
      fsm::arm.receive(fsm::event::GripperCommand{false});
    }
    if (new_value > 0 && new_value < 500) {  // SG在中
      fsm::arm.receive(fsm::event::GripperCommand{true});
    }
    if (new_value > 500 && new_value < 1100) {  // SG在内
      fsm::arm.receive(fsm::event::GripperCommand{true});
    }
  });
}