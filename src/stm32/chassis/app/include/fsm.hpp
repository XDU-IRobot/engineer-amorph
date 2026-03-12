
#pragma once

#include <librm.hpp>
#include <etl/fsm.h>

#include "globals.hpp"

namespace fsm {

struct EventId {
  enum {
    kControlLoop,      ///< 控制循环时钟事件
    kForceModeSwitch,  ///< 切换模式命令
    kFaultClear,       ///< 故障清除命令
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kNoForce,  ///< 无力
    kNormal,   ///< 有力
    kFault,    ///< 故障
  };
};

namespace event {
/*******************************************/
struct ControlLoop : etl::message<EventId::kControlLoop> {
  float f_steer_pos;
  float f_steer_vel;
  float l_steer_pos;
  float l_steer_vel;
  float r_steer_pos;
  float r_steer_vel;
  float f_wheel_vel;
  float l_wheel_vel;
  float r_wheel_vel;
};

/*******************************************/
struct ForceModeSwitch : etl::message<EventId::kForceModeSwitch> {
  explicit ForceModeSwitch(etl::fsm_state_id_t target) : target_mode(target) {}
  etl::fsm_state_id_t target_mode;
};

/*******************************************/
struct FaultClear : etl::message<EventId::kFaultClear> {};

}  // namespace event

class Chassis : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = 0;

 public:
  Chassis() : fsm(kMessageRouterId) {}
};
extern Chassis chassis;

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
struct NoForce : etl::fsm_state<Chassis, NoForce, StateId::kNoForce,  //
                                event::ForceModeSwitch,               //
                                event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    globals->controller.Enable(false);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    globals->actuator.f_steer->SetCurrent(0);
    globals->actuator.l_steer->SetCurrent(0);
    globals->actuator.r_steer->SetCurrent(0);
    globals->actuator.f_wheel->SetCurrent(0);
    globals->actuator.l_wheel->SetCurrent(0);
    globals->actuator.r_wheel->SetCurrent(0);
    return No_State_Change;
  }
};

/*******************************************/
struct Normal : etl::fsm_state<Chassis, Normal, StateId::kNormal,  //
                               event::ForceModeSwitch,             //
                               event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->buzzer_controller.Play<buzzer_melody::EnterManualMode>();
    globals->controller.Enable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    using RC = rm::device::RefereeProtocol<rm::device::RefereeRevision::kV170>::RemoteControl;
    const uint16_t keycode = globals->ref_vt_link.data().remote_control.keyboard_value;
    const float key_w = (keycode & RC::kW) ? 1.f : 0.f;
    const float key_s = (keycode & RC::kS) ? 1.f : 0.f;
    const float key_a = (keycode & RC::kA) ? 1.f : 0.f;
    const float key_d = (keycode & RC::kD) ? 1.f : 0.f;
    const float key_q = (keycode & RC::kQ) ? 1.f : 0.f;
    const float key_e = (keycode & RC::kE) ? 1.f : 0.f;
    const float target_vx = (globals->rc.left_x() + key_d - key_a) * 400.f;
    const float target_vy = (globals->rc.left_y() + key_w - key_s) * 400.f;
    const float target_w = -(globals->rc.right_x() + key_e - key_q) * 1000.f;
    globals->controller.SetTarget(target_vx, target_vy, target_w);
    globals->controller.Update(e.f_steer_pos, e.f_steer_vel,  //
                               e.l_steer_pos, e.l_steer_vel,  //
                               e.r_steer_pos, e.r_steer_vel,  //
                               e.f_wheel_vel, e.l_wheel_vel, e.r_wheel_vel);
    const auto &control = globals->controller.output();
    globals->actuator.f_steer->SetCurrent(-control.front_steer);
    globals->actuator.l_steer->SetCurrent(-control.lr_steer);
    globals->actuator.r_steer->SetCurrent(-control.rr_steer);
    globals->actuator.f_wheel->SetCurrent(-control.front_wheel);
    globals->actuator.l_wheel->SetCurrent(-control.lr_wheel);
    globals->actuator.r_wheel->SetCurrent(-control.rr_wheel);
    // globals->actuator.l_wheel->SetCurrent(0);
    // globals->actuator.r_wheel->SetCurrent(0);
    rm::device::DjiMotorBase::SendCommand();
    return No_State_Change;
  }
};

/*******************************************/
struct Fault : etl::fsm_state<Chassis, Fault, StateId::kFault,  //
                              event::ControlLoop,               //
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

}  // namespace fsm