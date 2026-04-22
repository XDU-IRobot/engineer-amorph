
#pragma once

#include <etl/fsm.h>

#include "globals.hpp"

namespace fsm_gimbal {

struct EventId {
  enum {
    kControlLoop,      ///< 控制循环时钟事件
    kKeyboardCommand,  ///< 客户端键盘命令
    kForceModeSwitch,  ///< 切换模式命令
    kFaultClear,       ///< 故障清除命令
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kNoForce,  ///< 无力
    kNormal,   ///< 有力
    kManual,   ///< 遥控器手动模式
    kFault,    ///< 故障
  };
};

namespace event {
/*******************************************/
struct ControlLoop : etl::message<EventId::kControlLoop> {};

/*******************************************/
struct KeyboardCommand : etl::message<EventId::kKeyboardCommand> {
  explicit KeyboardCommand(bool up) : up{up} {}
  bool up;  ///< 杆升？
};

/*******************************************/
struct ForceModeSwitch : etl::message<EventId::kForceModeSwitch> {
  explicit ForceModeSwitch(etl::fsm_state_id_t target) : target_mode(target) {}
  etl::fsm_state_id_t target_mode;
};

/*******************************************/
struct FaultClear : etl::message<EventId::kFaultClear> {};

}  // namespace event

class Gimbal : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = 1;

 public:
  Gimbal() : fsm(kMessageRouterId) {}

  static void Cmd(bool up) {
    if (up) {
      globals->actuator.camera_lift->SetMitCommand(0, 0, 6.5f, 0, 0.0f);
    } else {
      globals->actuator.camera_lift->SetMitCommand(0, 0, -0.3f, 0, 0.009f);
    }
  }
};
extern Gimbal gimbal;

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
struct NoForce : etl::fsm_state<Gimbal, NoForce, StateId::kNoForce,  //
                                event::ForceModeSwitch,              //
                                event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    globals->actuator.camera_lift->SetMitCommand(0, 0, 0, 0, 0);
    return No_State_Change;
  }
};

/*******************************************/
struct Normal : etl::fsm_state<Gimbal, Normal, StateId::kNormal,  //
                               event::ForceModeSwitch,            //
                               event::KeyboardCommand,            //
                               event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->buzzer_controller.Play<buzzer_melody::EnterManualMode>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) { return No_State_Change; }
  REACT(event::KeyboardCommand) {
    if (e.up) {
      Gimbal::Cmd(true);
    } else {
      Gimbal::Cmd(false);
    }
    return No_State_Change;
  }
};

/*******************************************/
struct Manual : etl::fsm_state<Gimbal, Manual, StateId::kManual,  //
                               event::ForceModeSwitch,            //
                               event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->buzzer_controller.Play<buzzer_melody::EnterManualMode>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    if (globals->rc.channel(rc_ch::SF) < 1000) {
      Gimbal::Cmd(true);
    } else {
      Gimbal::Cmd(false);
    }
    return No_State_Change;
  }
};

/*******************************************/
struct Fault : etl::fsm_state<Gimbal, Fault, StateId::kFault,  //
                              event::ControlLoop,              //
                              event::FaultClear> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
    globals->controller.Enable(false);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    globals->actuator.f_steer->SetCurrent(0);
    globals->actuator.l_steer->SetCurrent(0);
    globals->actuator.r_steer->SetCurrent(0);
    globals->actuator.f_wheel->SetCurrent(0);
    globals->actuator.l_wheel->SetCurrent(0);
    globals->actuator.r_wheel->SetCurrent(0);
    return No_State_Change;
  }
  REACT(event::FaultClear) {
    UNUSED(e);
    return StateId::kNoForce;
  }
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm_gimbal