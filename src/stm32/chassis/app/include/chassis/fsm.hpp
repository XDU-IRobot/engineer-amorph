
#pragma once

#include <etl/fsm.h>

#include "resources.hpp"
#include "shared_resources.hpp"

namespace fsm_chassis {

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
  float f_wheel_current;
  float l_wheel_current;
  float r_wheel_current;
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
  Chassis(ChassisResources &r) : fsm(kMessageRouterId), resources_(&r) {}

  ChassisResources &resources() const { return *resources_; }

 private:
  ChassisResources *resources_;
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
    auto &shared = SharedResources::GetInstance();
    shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    get_fsm_context().resources().controller.Enable(false);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();
    resources.f_steer.SetCurrent(0);
    resources.l_steer.SetCurrent(0);
    resources.r_steer.SetCurrent(0);
    resources.f_wheel.SetCurrent(0);
    resources.l_wheel.SetCurrent(0);
    resources.r_wheel.SetCurrent(0);
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
    SharedResources::GetInstance().buzzer_controller.Play<buzzer_melody::EnterManualMode>();
    get_fsm_context().resources().controller.Enable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    using RC = rm::device::RefereeProtocol<rm::device::RefereeRevision::kV170>::RemoteControl;
    const uint16_t keycode = shared.ref_vt_link.data().remote_control.keyboard_value;
    const bool move_gently = (keycode & RC::kShift) != 0;  // 按住Ctrl键，慢速移动
    const float slow_mod = move_gently ? 0.3f : 1.f;
    const float key_w = ((keycode & RC::kW) ? 1.f : 0.f) * slow_mod;
    const float key_s = ((keycode & RC::kS) ? 1.f : 0.f) * slow_mod;
    const float key_a = ((keycode & RC::kA) ? 1.f : 0.f) * slow_mod;
    const float key_d = ((keycode & RC::kD) ? 1.f : 0.f) * slow_mod;
    const float key_q = ((keycode & RC::kQ) ? 1.f : 0.f) * slow_mod;
    const float key_e = ((keycode & RC::kE) ? 1.f : 0.f) * slow_mod;
    const float target_vx = (shared.rc.left_x() + key_d - key_a) * 1000.f;
    const float target_vy = (shared.rc.left_y() + key_w - key_s) * 1000.f;
    const float target_w = -(shared.rc.right_x() + key_e - key_q) * 3000.f;
    resources.controller.SetTarget(target_vx, target_vy, target_w);
    resources.controller.Update(e.f_steer_pos, e.f_steer_vel,                             //
                                e.l_steer_pos, e.l_steer_vel,                             //
                                e.r_steer_pos, e.r_steer_vel,                             //
                                e.f_wheel_vel, e.l_wheel_vel, e.r_wheel_vel,              //
                                e.f_wheel_current, e.l_wheel_current, e.r_wheel_current,  //
                                move_gently);
    const auto &control = resources.controller.output();
    resources.f_steer.SetCurrent(-control.front_steer);
    resources.l_steer.SetCurrent(-control.lr_steer);
    resources.r_steer.SetCurrent(-control.rr_steer);
    resources.f_wheel.SetCurrent(-control.front_wheel);
    resources.l_wheel.SetCurrent(-control.lr_wheel);
    resources.r_wheel.SetCurrent(-control.rr_wheel);
    return No_State_Change;
  }
};

/*******************************************/
struct Fault : etl::fsm_state<Chassis, Fault, StateId::kFault,  //
                              event::ControlLoop,               //
                              event::FaultClear> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    SharedResources::GetInstance().buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
    get_fsm_context().resources().controller.Enable(false);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    auto &resources = get_fsm_context().resources();
    resources.f_steer.SetCurrent(0);
    resources.l_steer.SetCurrent(0);
    resources.r_steer.SetCurrent(0);
    resources.f_wheel.SetCurrent(0);
    resources.l_wheel.SetCurrent(0);
    resources.r_wheel.SetCurrent(0);
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

}  // namespace fsm_chassis