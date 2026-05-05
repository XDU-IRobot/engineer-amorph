
#pragma once

#include <etl/fsm.h>

#include "shared_resources.hpp"

namespace fsm_gimbal {

struct EventId {
  enum {
    kControlLoop,      ///< 控制循环时钟事件
    kForceModeSwitch,  ///< 切换模式命令
    kFaultClear,       ///< 故障清除命令
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kNoForce,    ///< 无力
    kUp,         ///< 有力，抬起
    kMid,        ///< 有力，保持在底盘中心+云台yaw自稳
    kDown,       ///< 有力，放下，根据底盘imu抬起一定角度
    kForceDown,  ///< 有力，放下到最低处
    kFault,      ///< 故障
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
struct FaultClear : etl::message<EventId::kFaultClear> {};

}  // namespace event

class Gimbal : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = 1;

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
    SharedResources::GetInstance().buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();
    resources.pitch_servo.SetLoad(true);
    resources.yaw_motor.SetCurrent(0);
    return No_State_Change;
  }
};

/*******************************************/
struct Up : etl::fsm_state<Gimbal, Up, StateId::kUp,  //
                           event::ForceModeSwitch,    //
                           event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &resources = get_fsm_context().resources();
    const float current_yaw = resources.yaw_calib.RealToLogical(resources.yaw_motor.pos_rad());
    resources.encoder_yaw_pid.Clear();
    yaw_planner_.ResetAt(current_yaw);
    yaw_planner_.SetTarget(0.f);  // 回中
    resources.pitch_servo.MoveTime(resources.config.servo_mid_angle, resources.config.servo_move_time_ms);
    SharedResources::GetInstance()
        .buzzer_controller.Play<rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kE6>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();
    const float current_yaw = resources.yaw_calib.RealToLogical(resources.yaw_motor.pos_rad());
    yaw_planner_.Update(resources.config.control_loop_dt);
    resources.encoder_yaw_pid.UpdateExtDiff(yaw_planner_.current_position(), current_yaw,
                                            resources.yaw_motor.rpm() * 2.f * M_PI / 60.f,  // to rad/s
                                            resources.config.control_loop_dt);
    resources.yaw_motor.SetCurrent(-resources.encoder_yaw_pid.out());
    return No_State_Change;
  }

 private:
  rm::modules::TrajectoryLimiter yaw_planner_{50.f, 100.f};
};

/*******************************************/
struct Mid : etl::fsm_state<Gimbal, Mid, StateId::kMid,  //
                            event::ForceModeSwitch,      //
                            event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &resources = get_fsm_context().resources();
    target_yaw_ = resources.imu.yaw();
    resources.imu_yaw_pid.Clear();
    resources.pitch_servo.MoveTime(resources.config.servo_mid_angle, resources.config.servo_move_time_ms);
    SharedResources::GetInstance()
        .buzzer_controller.Play<rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kD6>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();
    resources.imu_yaw_pid.UpdateExtDiff(target_yaw_, resources.imu.yaw(), -resources.imu.gyro_z(),
                                        resources.config.control_loop_dt);
    resources.yaw_motor.SetCurrent(-resources.imu_yaw_pid.out());
    return No_State_Change;
  }

 private:
  float target_yaw_{0.f};
};

/*******************************************/
struct Down : etl::fsm_state<Gimbal, Down, StateId::kDown,  //
                             event::ForceModeSwitch,        //
                             event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    SharedResources::GetInstance()
        .buzzer_controller.Play<rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kC6>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) { return No_State_Change; }
};

/*******************************************/
struct ForceDown : etl::fsm_state<Gimbal, ForceDown, StateId::kForceDown,  //
                                  event::ForceModeSwitch,                  //
                                  event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    SharedResources::GetInstance().buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) { return No_State_Change; }
};

/*******************************************/
struct Fault : etl::fsm_state<Gimbal, Fault, StateId::kFault,  //
                              event::ControlLoop,              //
                              event::FaultClear> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    SharedResources::GetInstance().buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    auto &resources = get_fsm_context().resources();
    resources.yaw_motor.SetCurrent(0);
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