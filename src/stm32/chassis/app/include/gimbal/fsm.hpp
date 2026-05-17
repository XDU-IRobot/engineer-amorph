
#pragma once

#include <etl/fsm.h>
#include <etl/debounce.h>

#include "shared_resources.hpp"

namespace fsm_gimbal {

struct EventId {
  enum {
    kControlLoop,  ///< 控制循环时钟事件
    kModeSwitch,   ///< 切换模式命令
    kFaultClear,   ///< 故障清除命令
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kNoForce,               ///< 无力
    kUp,                    ///< 有力，抬起
    kMid,                   ///< 有力，保持在底盘中心+云台yaw自稳
    kDown,                  ///< 有力，放下，根据底盘imu抬起一定角度
    kForceDown,             ///< 有力，放下到最低处
    kLongJointCalibrating,  ///< 图传杆编码器校准
    kFault,                 ///< 故障
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
  etl::fsm_state_id_t on_event(const event::ModeSwitch &e) { return e.target_mode; }
#define IGNORE_UNINTEREST_EVENT() \
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &) { return No_State_Change; }
#define ENTER etl::fsm_state_id_t on_enter_state()
#define REACT(EventType) etl::fsm_state_id_t on_event(const EventType &e)
///*********************************
///*********************************
///*********************************

/*******************************************/
struct NoForce : etl::fsm_state<Gimbal, NoForce, StateId::kNoForce,  //
                                event::ModeSwitch,                   //
                                event::ControlLoop> {
  // ACCEPT_MODE_SWITCH();
  REACT(event::ModeSwitch) {
    auto &resources = get_fsm_context().resources();

    if (resources.lift_calibrated) {
      return e.target_mode;
    }
    if (e.target_mode == StateId::kUp ||    //
        e.target_mode == StateId::kMid ||   //
        e.target_mode == StateId::kDown ||  //
        e.target_mode == StateId::kForceDown) {
      return StateId::kLongJointCalibrating;
    }

    return No_State_Change;
  }
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &resources = get_fsm_context().resources();

    SharedResources::GetInstance().buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    resources.lift_motor.SendInstruction(rm::device::Ht04Motor::Instruction::kDisable);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();
    resources.pitch_servo.SetLoad(true);
    resources.yaw_motor.SetCurrent(0);
    resources.lift_motor.SetMitCommand(0, 0, 0, 0, 0);
    return No_State_Change;
  }
};

/*******************************************/
struct Up : etl::fsm_state<Gimbal, Up, StateId::kUp,  //
                           event::ModeSwitch,         //
                           event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &resources = get_fsm_context().resources();

    const float current_yaw = resources.yaw_calib.RealToLogical(resources.yaw_motor.pos_rad());
    resources.encoder_yaw_pid.Clear();
    resources.yaw_tl.ResetAt(current_yaw);
    resources.yaw_tl.SetTarget(0.f);  // 回中
    resources.lift_motor.SendInstruction(rm::device::Ht04Motor::Instruction::kEnable);
    resources.pitch_servo.MoveTime(resources.config.servo_mid_angle, resources.config.servo_move_time_ms);
    resources.lift_tl.ResetAt(resources.camera_lift_calib.RealToLogical(resources.lift_motor.pos()));
    resources.lift_tl.SetTarget(0.f);
    SharedResources::GetInstance()
        .buzzer_controller.Play<rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kE6>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();
    const float current_yaw = resources.yaw_calib.RealToLogical(resources.yaw_motor.pos_rad());
    resources.yaw_tl.Update(resources.config.control_loop_dt);
    resources.encoder_yaw_pid.UpdateExtDiff(resources.yaw_tl.current_position(), current_yaw,
                                            resources.yaw_motor.rpm() * 2.f * M_PI / 60.f,  // to rad/s
                                            resources.config.control_loop_dt);
    resources.yaw_motor.SetCurrent(-resources.encoder_yaw_pid.out());
    resources.lift_tl.Update(0.004f);
    if (resources.lift_tl.IsAtTarget()) {
      resources.lift_motor.SetMitCommand(0, 0, -10.f, 0, 10);
    } else {
      resources.lift_motor.SetMitCommand(
          resources.camera_lift_calib.LogicalToReal(resources.lift_tl.current_position()), 0, 0.f,
          resources.config.lift_pd_param.kp, resources.config.lift_pd_param.kd);
    }
    return No_State_Change;
  }
};

/*******************************************/
struct Mid : etl::fsm_state<Gimbal, Mid, StateId::kMid,  //
                            event::ModeSwitch,           //
                            event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &resources = get_fsm_context().resources();
    target_yaw_ = resources.imu.yaw();
    resources.imu_yaw_pid.Clear();
    resources.lift_motor.SendInstruction(rm::device::Ht04Motor::Instruction::kEnable);
    resources.pitch_servo.MoveTime(resources.config.servo_mid_angle, resources.config.servo_move_time_ms);
    resources.lift_tl.ResetAt(resources.camera_lift_calib.RealToLogical(resources.lift_motor.pos()));
    resources.lift_tl.SetTarget(0.324966431f);
    SharedResources::GetInstance()
        .buzzer_controller.Play<rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kD6>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();
    resources.imu_yaw_pid.UpdateExtDiff(target_yaw_, resources.imu.yaw(), -resources.imu.gyro_z(),
                                        resources.config.control_loop_dt);
    resources.yaw_motor.SetCurrent(-resources.imu_yaw_pid.out());
    resources.lift_tl.Update(0.004f);
    resources.lift_motor.SetMitCommand(resources.camera_lift_calib.LogicalToReal(resources.lift_tl.current_position()),
                                       0, 0.f, resources.config.lift_pd_param.kp, resources.config.lift_pd_param.kd);
    return No_State_Change;
  }

 private:
  float target_yaw_{0.f};
};

/*******************************************/
struct Down : etl::fsm_state<Gimbal, Down, StateId::kDown,  //
                             event::ModeSwitch,             //
                             event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    using namespace rm::modules::angle_literals;
    auto &resources = get_fsm_context().resources();

    const float current_yaw = resources.yaw_calib.RealToLogical(resources.yaw_motor.pos_rad());
    resources.encoder_yaw_pid.Clear();
    resources.yaw_tl.ResetAt(current_yaw);
    resources.yaw_tl.SetTarget(0.f);  // 回中
    resources.lift_motor.SendInstruction(rm::device::Ht04Motor::Instruction::kEnable);
    resources.lift_tl.ResetAt(resources.camera_lift_calib.RealToLogical(resources.lift_motor.pos()));
    resources.lift_tl.SetTarget((90_deg).rad());
    SharedResources::GetInstance()
        .buzzer_controller.Play<rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kC6>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();
    const float current_yaw = resources.yaw_calib.RealToLogical(resources.yaw_motor.pos_rad());
    resources.yaw_tl.Update(resources.config.control_loop_dt);
    resources.encoder_yaw_pid.UpdateExtDiff(resources.yaw_tl.current_position(), current_yaw,
                                            resources.yaw_motor.rpm() * 2.f * M_PI / 60.f,  // to rad/s
                                            resources.config.control_loop_dt);
    resources.yaw_motor.SetCurrent(-resources.encoder_yaw_pid.out());
    resources.lift_tl.Update(0.004f);
    if (resources.lift_tl.IsAtTarget()) {
      resources.lift_motor.SetMitCommand(0, 0, 10.f, 0, 10);
    } else {
      resources.lift_motor.SetMitCommand(
          resources.camera_lift_calib.LogicalToReal(resources.lift_tl.current_position()), 0, 0.f,
          resources.config.lift_pd_param.kp, resources.config.lift_pd_param.kd);
    }
    return No_State_Change;
  }
};

/*******************************************/
struct ForceDown : etl::fsm_state<Gimbal, ForceDown, StateId::kForceDown,  //
                                  event::ModeSwitch,                       //
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
struct LongJointCalibrating : etl::fsm_state<Gimbal, LongJointCalibrating, StateId::kLongJointCalibrating,  //
                                             event::ModeSwitch,                                             //
                                             event::ControlLoop> {
 private:
  constexpr static int kStallThres = 125;  // 500ms
  constexpr static float kStallVelThres = 0.1f;
  using Debounce = etl::debounce<kStallThres>;

 public:
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &resources = get_fsm_context().resources();
    enter_state_timestamp_ = HAL_GetTick();
    is_motor_stalled_ = Debounce{};
    target_yaw_ = resources.imu.yaw();
    resources.lift_motor.SendInstruction(rm::device::Ht04Motor::Instruction::kEnable);
    SharedResources::GetInstance()
        .buzzer_controller.Play<rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kF6>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();
    camera_lift_vel_ = camera_lift_vel_lpf_.apply(resources.lift_motor.vel());
    is_motor_stalled_raw_ =
        std::abs(resources.lift_motor.torque()) > 25.f && std::abs(camera_lift_vel_) < kStallVelThres;
    is_motor_stalled_.add(is_motor_stalled_raw_);
    if (is_motor_stalled_.is_set()) {
      resources.camera_lift_calib.UpdateConfig(
          {.reverse = false, .zero_offset = resources.lift_motor.pos() + (6.5_deg).rad()});
      resources.lift_calibrated = true;
      return StateId::kUp;
    }

    resources.yaw_motor.SetCurrent(0);
    resources.lift_motor.SetMitCommand(0, 0, -40.f, 0, 20.f);
    return No_State_Change;
  }

 private:
  uint32_t enter_state_timestamp_{};
  rm::modules::LowPassFilterConstDt<float> camera_lift_vel_lpf_{250.f, 4.f};
  float camera_lift_vel_{};
  bool is_motor_stalled_raw_{false};
  Debounce is_motor_stalled_{};
  float target_yaw_{0.f};
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