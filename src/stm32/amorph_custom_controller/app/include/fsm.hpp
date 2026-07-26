#pragma once

#include <cmath>

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

  ArmDynamics dynamic_model{{.b1 = 0.2646f, .b2 = 0.1094f}};
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
/**
 * @brief Marker class that explicitly disables a force operation.
 *
 * This class provides a type-safe alternative to a boolean \c false passed
 * to methods or constructors that accept an optional force flag. By using
 * an instance of NoForce, the intent is made clear and the code becomes
 * more self-documenting.
 *
 * @details When a parameter can accept either a boolean force indicator or
 * an object like NoForce, passing an instance of this class guarantees that
 * no forced behaviour will be executed. It prevents accidental misuse of
 * raw boolean values and improves readability, especially in contexts where
 * multiple boolean parameters might otherwise be confusing.
 */

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
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kC6>>();
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
    globals->actuator.j2.SetMitCommand(0, 0, 0, 0, 0);
    globals->actuator.j3.SetMitCommand(0, 0, 0, 0, 0);
    globals->actuator.j4.SetPower(0);
    globals->actuator.j5.SetPower(0);
    globals->actuator.j6.SetPower(0);
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
    target_pos_ = globals->actuator.GetRawPositions();
    globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    return No_State_Change;
  }
  REACT(event::ButtonEvent) {
    switch (e.button_idx) {
      case Buttons::kOnBoardKey:
        return StateId::kNoForce;
      default:
        return No_State_Change;
    }
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    if (globals->buttons.b()) {
      return StateId::kEnabled;
    }

    current_pos_ = {globals->actuator.GetPositions()[0],  //
                    globals->actuator.GetPositions()[1],  //
                    globals->actuator.GetPositions()[2]};
    compensate_t_ = get_fsm_context().dynamic_model.Compensate(current_pos_, {0.f, 0.f, -9.81f});
    globals->actuator.j1.SetCurrent(0);
    // globals->actuator.j2.SetMitCommand(target_pos[1], 0, 0, 20, 0.4);
    // globals->actuator.j3.SetMitCommand(target_pos[2], 0, 0, 15, 2);
    const float j2_rev = globals->actuator.calibrators[1].reverse() ? -1.f : 1.f;
    const float j3_rev = globals->actuator.calibrators[2].reverse() ? -1.f : 1.f;
    globals->actuator.j2.SetMitCommand(target_pos_[1], 0, j2_rev * std::clamp(compensate_t_(1), -5.f, 5.f), 2, 0.5);
    globals->actuator.j3.SetMitCommand(target_pos_[2], 0, j3_rev * std::clamp(compensate_t_(2), -5.f, 5.f), 7, 2);
    globals->actuator.j4.SetSpeed(0);
    globals->actuator.j5.SetSpeed(0);
    globals->actuator.j6.SetSpeed(0);
    rm::device::DjiMotorBase::SendCommand();
    return No_State_Change;
  }

  std::array<float, 6> target_pos_{};
  Eigen::Vector3f current_pos_;
  Eigen::Vector3f compensate_t_;
};

/*******************************************/
struct Enabled : etl::fsm_state<CcArm, Enabled, StateId::kEnabled,  //
                                event::ButtonEvent,                 //
                                event::ForceModeSwitch,             //
                                event::ControlLoop> {
  Enabled() {
    btn_a_svw_.OnValueChange([this](const bool &old_value, const bool &new_value) {
      if (!old_value && new_value) {
        globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
      }
    });
  }
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    j4_center_pid_.Clear();
    globals->actuator.SetEnable(true);
    globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    return No_State_Change;
  }
  REACT(event::ButtonEvent) {
    switch (e.button_idx) {
      case Buttons::kOnBoardKey:
        return StateId::kNoForce;
      default:
        return No_State_Change;
    }
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    if (!globals->buttons.b()) {
      return StateId::kLockPosition;
    }

    btn_a_svw_.Update(globals->buttons.a());

    current_vel_ = globals->actuator.GetVelocities();
    boost_torque_[1] = std::tanhf(current_vel_[1]);
    boost_torque_[2] = std::tanhf(0.6f * current_vel_[2]);
    current_pos_ = {globals->actuator.GetPositions()[0],  //
                    globals->actuator.GetPositions()[1],  //
                    globals->actuator.GetPositions()[2]};
    compensate_torque_ = get_fsm_context().dynamic_model.Compensate(current_pos_, {0.f, 0.f, -9.81f});
    globals->actuator.j1.SetCurrent(0);
    // globals->actuator.j2.SetMitCommand(target_pos[1], 0, 0, 20, 0.4);
    // globals->actuator.j3.SetMitCommand(target_pos[2], 0, 0, 15, 2);
    j4_center_pid_.Update(0.f, globals->actuator.GetPositions()[3], 1.f / 250.f);
    const float j2_rev = globals->actuator.calibrators[1].reverse() ? -1.f : 1.f;
    const float j3_rev = globals->actuator.calibrators[2].reverse() ? -1.f : 1.f;
    globals->actuator.j2.SetMitCommand(
        0, 0, j2_rev * std::clamp(compensate_torque_(1) + (boost_torque_[1] * 1.f), -5.f, 5.f), 0, 0);
    globals->actuator.j3.SetMitCommand(
        0, 0, j3_rev * std::clamp(compensate_torque_(2) + (boost_torque_[2] * 0.3f), -5.f, 5.f), 0, 0);
    globals->actuator.j4.SetPower(j4_center_pid_.out());
    globals->actuator.j5.SetPower(0);
    globals->actuator.j6.SetPower(0);
    rm::device::DjiMotorBase::SendCommand();
    return No_State_Change;
  }

 private:
  float boost_torque_[3]{};
  std::array<float, 6> current_vel_{};
  Eigen::Vector3f current_pos_;
  Eigen::Vector3f compensate_torque_;
  rm::modules::PID j4_center_pid_{100.f, 0.f, 0.f, 70.f, 0.f};
  rm::modules::SparseValueWatcher<bool, true> btn_a_svw_;
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm