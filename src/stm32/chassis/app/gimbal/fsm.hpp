
#pragma once

#include <etl/fsm.h>

#include "shared_resources.hpp"

namespace fsm_gimbal {

struct EventId {
  enum {
    kControlLoop,   ///< 控制循环时钟事件
    kModeSwitch,    ///< 切换模式命令
    kPitchCommand,  ///<
    kFaultClear,    ///< 故障清除命令
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kNoForce,  ///< 无力
    kUp,       ///< 有力，抬起
    kDown,     ///< 有力，放下
    NUMBER_OF_STATES,
  };
};

namespace event {
/*******************************************/
struct ControlLoop : etl::message<EventId::kControlLoop> {};

/*******************************************/
struct ModeSwitch : etl::message<EventId::kModeSwitch> {
  explicit ModeSwitch(etl::fsm_state_id_t target) : target_mode(target) {}
  etl::fsm_state_id_t target_mode;
};

/*******************************************/
struct PitchCommand : etl::message<EventId::kPitchCommand> {
  explicit PitchCommand(bool up) : up(up) {}
  bool up;
};

/*******************************************/
struct FaultClear : etl::message<EventId::kFaultClear> {};

}  // namespace event

class Gimbal : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = GlobalConfig::kGimbalMessageRouterId;

 public:
  explicit Gimbal(GimbalResources &resources) : fsm(kMessageRouterId), resources_(&resources) {}

  GimbalResources &resources() const { return *resources_; }

 private:
  GimbalResources *resources_;
};
extern Gimbal gimbal;

namespace state {

///*********************************
///*******  helper macros  *********
///*********************************
#define ACCEPT_MODE_SWITCH() \
  etl::fsm_state_id_t on_event(const event::ModeSwitch &e) { return e.target_mode; }
#define IGNORE_UNINTEREST_EVENT() \
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &) { return No_State_Change; }
#define ENTER etl::fsm_state_id_t on_enter_state()
#define REACT(EventType) etl::fsm_state_id_t on_event(const EventType &e)
///*********************************
///*********************************
///*********************************

/*******************************************/
/// @brief 所有电机无力
struct NoForce : etl::fsm_state<Gimbal, NoForce, StateId::kNoForce,  //
                                event::ModeSwitch,                   //
                                event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &resources = get_fsm_context().resources();

    SharedResources::GetInstance().buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    resources.lift_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();
    resources.lift_motor.SetMitCommand(0, 0, 0, 0, 0);
    resources.pitch_motor.SetCurrent(0);
    return No_State_Change;
  }
};

/*******************************************/
/// @brief 图传杆升起
struct Up : etl::fsm_state<Gimbal, Up, StateId::kUp,  //
                           event::ModeSwitch,         //
                           event::ControlLoop,        //
                           event::PitchCommand> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &resources = get_fsm_context().resources();

    resources.lift_tl.ResetAt(resources.lift_motor.pos());
    resources.lift_tl.SetTarget(0.f);
    resources.lift_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
    resources.current_pitch_command_up = true;
    SharedResources::GetInstance()
        .buzzer_controller.Play<rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kE6>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();

    resources.lift_tl.Update(0.004f);
    if (resources.lift_tl.IsAtTarget()) {
      resources.lift_motor.SetMitCommand(
          resources.camera_lift_calib.LogicalToReal(resources.lift_tl.current_position()), 0, -4.f,
          resources.config.lift_pd_param.kp, resources.config.lift_pd_param.kd);
    } else {
      resources.lift_motor.SetMitCommand(
          resources.camera_lift_calib.LogicalToReal(resources.lift_tl.current_position()),
          resources.lift_tl.current_velocity(), -5.f, resources.config.lift_pd_param.kp,
          resources.config.lift_pd_param.kd);
    }

    if (resources.current_pitch_command_up) {
      resources.pitch_motor.SetCurrent(1000);
    } else {
      resources.pitch_motor.SetCurrent(-2000);
    }
    return No_State_Change;
  }
  REACT(event::PitchCommand) {
    auto &resources = get_fsm_context().resources();

    if (e.up) {
      resources.current_pitch_command_up = true;
    } else {
      resources.current_pitch_command_up = false;
    }
    return No_State_Change;
  }
};

/*******************************************/
/// @brief 图传杆降到低处
struct Down : etl::fsm_state<Gimbal, Down, StateId::kDown,  //
                             event::ModeSwitch,             //
                             event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &resources = get_fsm_context().resources();
    using namespace rm::modules::angle_literals;

    resources.lift_tl.ResetAt(resources.lift_motor.pos());
    resources.lift_tl.SetTarget((90_deg).rad());
    resources.lift_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
    resources.current_pitch_command_up = true;
    SharedResources::GetInstance()
        .buzzer_controller.Play<rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kE6>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();

    resources.lift_tl.Update(0.004f);
    if (resources.lift_tl.IsAtTarget()) {
      resources.lift_motor.SetMitCommand(
          resources.camera_lift_calib.LogicalToReal(resources.lift_tl.current_position()), 0, 3.f,
          resources.config.lift_pd_param.kp, resources.config.lift_pd_param.kd);
    } else {
      resources.lift_motor.SetMitCommand(
          resources.camera_lift_calib.LogicalToReal(resources.lift_tl.current_position()),
          resources.lift_tl.current_velocity(), -8.f, resources.config.lift_pd_param.kp,
          resources.config.lift_pd_param.kd);
    }

    resources.pitch_motor.SetCurrent(1000);

    return No_State_Change;
  }
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm_gimbal