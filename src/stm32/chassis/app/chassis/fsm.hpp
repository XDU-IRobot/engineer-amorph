
#pragma once

#include <etl/fsm.h>

#include "resources.hpp"
#include "shared_resources.hpp"

namespace fsm_chassis {

struct EventId {
  enum {
    kControlLoop,      ///< 控制循环时钟事件
    kForceModeSwitch,  ///< 切换模式命令
    kVelocityCommand,  ///< 底盘速度命令
    kFaultClear,       ///< 故障清除命令
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kNoForce,  ///< 无力
    kDebug,    ///< 有力，调试模式
    kNormal,   ///< 有力，比赛模式
    kFault,    ///< 故障
    NUMBER_OF_STATES,
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
  float f_steer_current;
  float l_steer_current;
  float r_steer_current;
};

/*******************************************/
struct ForceModeSwitch : etl::message<EventId::kForceModeSwitch> {
  explicit ForceModeSwitch(etl::fsm_state_id_t target) : target_mode(target) {}
  etl::fsm_state_id_t target_mode;
};

/*******************************************/
struct VelocityCommand : etl::message<EventId::kVelocityCommand> {
  VelocityCommand() : timestamp(HAL_GetTick()) {}
  VelocityCommand(float vx, float vy, float w) : timestamp(HAL_GetTick()), vx(vx), vy(vy), w(w) {}
  uint32_t timestamp;
  float vx{};
  float vy{};
  float w{};
  bool move_gently{false};
};

/*******************************************/
struct FaultClear : etl::message<EventId::kFaultClear> {};

}  // namespace event

class Chassis : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = GlobalConfig::kChassisMessageRouterId;

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
    // shared.buzzer_controller.Play<rm::modules::buzzer_melody::SuperMario>();
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
struct Debug : etl::fsm_state<Chassis, Debug, StateId::kDebug,  //
                              event::ForceModeSwitch,           //
                              event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();
    SharedResources::GetInstance().buzzer_controller.Play<buzzer_melody::EnterManualMode>();
    get_fsm_context().resources().controller.Enable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    using RC = rm::device::VT03::KeyboardKey;
    const uint16_t keycode = shared.vt_rc.data().keyboard_key;
    const bool move_gently = (keycode & RC::kShift) != 0;  // 按住Shift键，慢速移动
    const float slow_mod = move_gently ? 0.07f : 1.f;
    const float key_w = ((keycode & RC::kW) ? 1.f : 0.f) * slow_mod;
    const float key_s = ((keycode & RC::kS) ? 1.f : 0.f) * slow_mod;
    const float key_a = ((keycode & RC::kA) ? 1.f : 0.f) * slow_mod;
    const float key_d = ((keycode & RC::kD) ? 1.f : 0.f) * slow_mod;
    const float key_q = ((keycode & RC::kC) ? 1.f : 0.f) * slow_mod;
    const float key_e = ((keycode & RC::kB) ? 1.f : 0.f) * slow_mod;

    // 遥控器
    const float rc_x = shared.rc.left_x();
    const float rc_y = shared.rc.left_y();
    const float rc_w = shared.rc.right_x();

    // 选手端键盘
    const float kbd_x = key_d - key_a;
    const float kbd_y = key_w - key_s;
    const float kbd_w = 0.f;

    // 选手端鼠标
    const float vt_w = shared.vt_rc.data().mouse_x / 660.f;

    // 自定义客户端 xbox手柄
    const auto &client_data = shared.xbox_controller.data();
    const float client_x = client_data.left_x;
    const float client_y = client_data.left_y;
    const float client_w = client_data.right_x + client_data.lt - client_data.rt;

    const float target_vx = (rc_x + kbd_x + client_x) * 1000.f;
    const float target_vy = (rc_y + kbd_y + client_y) * 1000.f;
    const float target_w = -(rc_w + kbd_w + client_w + vt_w) * 5.f;
    resources.controller.SetTarget(target_vx, target_vy, target_w);
    resources.controller.Update(e.f_steer_pos, e.f_steer_vel,                             //
                                e.l_steer_pos, e.l_steer_vel,                             //
                                e.r_steer_pos, e.r_steer_vel,                             //
                                e.f_wheel_vel, e.l_wheel_vel, e.r_wheel_vel,              //
                                e.f_wheel_current, e.l_wheel_current, e.r_wheel_current,  //
                                e.f_steer_current, e.l_steer_current, e.r_steer_current,  //
                                shared.ahrs.imu_data().gz,                                //
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
struct Normal : etl::fsm_state<Chassis, Normal, StateId::kNormal,  //
                               event::ForceModeSwitch,             //
                               event::VelocityCommand,             //
                               event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();
    shared.buzzer_controller.Play<buzzer_melody::EnterManualMode>();
    get_fsm_context().resources().controller.Enable(true);
    return No_State_Change;
  }
  REACT(event::VelocityCommand) {
    velocity_cmd_ = e;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    constexpr uint32_t kCommandStaleTimeThresh = 500;  // ms

    if (HAL_GetTick() - velocity_cmd_.timestamp > kCommandStaleTimeThresh) {
      velocity_cmd_ = event::VelocityCommand{0, 0, 0};
    }

    resources.controller.SetTarget(velocity_cmd_.vx, velocity_cmd_.vy, velocity_cmd_.w);
    resources.controller.Update(e.f_steer_pos, e.f_steer_vel,                             //
                                e.l_steer_pos, e.l_steer_vel,                             //
                                e.r_steer_pos, e.r_steer_vel,                             //
                                e.f_wheel_vel, e.l_wheel_vel, e.r_wheel_vel,              //
                                e.f_wheel_current, e.l_wheel_current, e.r_wheel_current,  //
                                e.f_steer_current, e.l_steer_current, e.r_steer_current,  //
                                shared.ahrs.imu_data().gz,                                //
                                velocity_cmd_.move_gently);
    const auto &control = resources.controller.output();
    resources.f_steer.SetCurrent(-control.front_steer);
    resources.l_steer.SetCurrent(-control.lr_steer);
    resources.r_steer.SetCurrent(-control.rr_steer);
    resources.f_wheel.SetCurrent(-control.front_wheel);
    resources.l_wheel.SetCurrent(-control.lr_wheel);
    resources.r_wheel.SetCurrent(-control.rr_wheel);
    return No_State_Change;
  }

 private:
  event::VelocityCommand velocity_cmd_;
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