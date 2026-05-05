#pragma once

#include <etl/fsm.h>

#include "dynamics.hpp"
#include "globals.hpp"

namespace fsm {

struct EventId {
  enum {
    kControlLoop,      ///< 控制循环时钟
    kForceModeSwitch,  ///< 切换模式命令
    kButtonEvent,      ///< 按钮按下
    kFaultClear,       ///< 故障清除命令
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kNoForce,       ///< 无力
    kLockPosition,  ///< 所有关节锁定到当前位置
    kEnabled,       ///< 使能，启动重力补偿，启动力反馈，可以控制机械臂
  };
};

namespace event {
/*******************************************/
struct ControlLoop : etl::message<EventId::kControlLoop> {};

/*******************************************/
struct ForceModeSwitch : etl::message<EventId::kForceModeSwitch> {
  explicit ForceModeSwitch(etl::fsm_state_id_t target) : target_mode(target) {}
  etl::fsm_state_id_t target_mode;
};

/*******************************************/
struct ButtonEvent : etl::message<EventId::kButtonEvent> {
  explicit ButtonEvent(int button_idx) : button_idx(button_idx) {}
  int button_idx;
};

/*******************************************/
struct FaultClear : etl::message<EventId::kFaultClear> {};

}  // namespace event

class CcArm : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = 0;

 public:
  explicit CcArm() : fsm(kMessageRouterId) {}

  ArmDynamics dynamic_model{{.b1 = 0.255138f, .b2 = 0.140616f}};
};
extern CcArm cc_arm;

namespace state {

///*********************************
///*******  helper macros  *********
///*********************************
#define ACCEPT_MODE_SWITCH() \
  etl::fsm_state_id_t on_event(const event::ForceModeSwitch &e) { return e.target_mode; }
#define IGNORE_UNINTEREST_EVENT() \
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &) { return No_State_Change; }
#define ENTER etl::fsm_state_id_t on_enter_state()
#define REACT(EventType) etl::fsm_state_id_t on_event(const EventType &e)
///*********************************
///*********************************
///*********************************

/*******************************************/
struct NoForce : etl::fsm_state<CcArm, NoForce, StateId::kNoForce,  //
                                event::ButtonEvent,                 //
                                event::ForceModeSwitch,             //
                                event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->actuator.SetEnable(false);
    globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    return No_State_Change;
  }
  REACT(event::ButtonEvent) {
    if (e.button_idx == Buttons::kOnBoardKey) {
      return StateId::kLockPosition;
    }
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    globals->actuator.SetEnable(false);
    return No_State_Change;
  }
};

/*******************************************/
struct LockPosition : etl::fsm_state<CcArm, LockPosition, StateId::kLockPosition,  //
                                     event::ButtonEvent,                           //
                                     event::ForceModeSwitch,                       //
                                     event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->actuator.SetEnable(true);
    target_pos[0] = globals->actuator.j1.pos_rad();
    target_pos[1] = globals->actuator.j2.pos();
    target_pos[2] = globals->actuator.j3.pos();
    globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    return No_State_Change;
  }
  REACT(event::ButtonEvent) {
    if (e.button_idx == Buttons::kOnBoardKey) {
      return StateId::kNoForce;
    }
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    current_pos = {globals->actuator.GetPositions()[0],  //
                   globals->actuator.GetPositions()[1],  //
                   globals->actuator.GetPositions()[2]};
    compensate_t = get_fsm_context().dynamic_model.Compensate(current_pos, {0.f, 0.f, -9.81f});
    globals->actuator.j1.SetCurrent(0);
    // globals->actuator.j2.SetMitCommand(target_pos[1], 0, 0, 20, 0.4);
    // globals->actuator.j3.SetMitCommand(target_pos[2], 0, 0, 15, 2);
    globals->actuator.j2.SetMitCommand(0, 0, -std::clamp(compensate_t(1), -5.f, 5.f), 0, 0);
    globals->actuator.j3.SetMitCommand(0, 0, std::clamp(compensate_t(2), -5.f, 5.f), 0, 0);
    globals->actuator.j4.SetSpeed(0);
    globals->actuator.j5.SetSpeed(0);
    globals->actuator.j6.SetSpeed(0);
    rm::device::DjiMotorBase::SendCommand();
    return No_State_Change;
  }

  float target_pos[3];
  Eigen::Vector3f current_pos;
  Eigen::Vector3f compensate_t;
};

/*******************************************/
struct Enabled : etl::fsm_state<CcArm, Enabled, StateId::kEnabled,  //
                                event::ButtonEvent,                 //
                                event::ForceModeSwitch,             //
                                event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER { return No_State_Change; }
  REACT(event::ButtonEvent) {
    if (e.button_idx == Buttons::kOnBoardKey) {
      return StateId::kNoForce;
    }
    return No_State_Change;
  }
  REACT(event::ControlLoop) { return No_State_Change; }
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm