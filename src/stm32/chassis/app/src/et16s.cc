
#include "et16s.hpp"

#include "globals.hpp"
#include "fsm.hpp"

void InitUserInput() {
  // 定义遥控器拨杆功能
  globals->rc(rc_ch::SD).OnValueChange([](const int &old_value, const int &new_value) {
    // 353-up, 1024-mid, 1695-down
    if (old_value == 0) {
      return;  // 第一次触发不响应
    }
    if (new_value > 1300 && new_value < 1700) {  // SD在下
      fsm::chassis.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});
    }
    if (new_value > 0 && new_value < 500) {  // SD在上
      fsm::chassis.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNormal});
    }
    if (new_value > 500 && new_value < 1100) {  // SD在中
      fsm::chassis.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNormal});
    }
  });
}