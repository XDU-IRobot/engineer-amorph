
#pragma once

#include <librm.hpp>
#include <etl/fsm.h>
#include <etl/cyclic_value.h>

#include "motion_planner.hpp"
#include "globals.hpp"

extern float caonima;

namespace fsm {

struct EventId {
  enum {
    kControlLoop,             ///< 控制循环时钟事件
    kForceModeSwitch,         ///< 切换模式命令
    kExecutionReq,            ///< 轨迹执行请求
    kGoToPosReq,              ///< 定点去往请求
    kCancelExecutionReq,      ///< 取消轨迹执行请求
    kGripperCommand,          ///< 夹爪控制命令
    kFaultClear,              ///< 故障清除命令
    kOpticalSwitchTriggered,  ///< 光电开关触发，j6回零完成
    kNextJoint,               ///< Manual模式下切换到下一个关节
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kNoForce,        ///< 强制无力
    kManual,         ///< 遥控器手动模式
    kIdle,           ///< 空闲，可执行
    kExecuting,      ///< 正在执行运动
    kJ6Calibrating,  ///< j6回零中
    kFault,          ///< 故障状态
  };
};

namespace event {
/*******************************************/
struct ControlLoop : etl::message<EventId::kControlLoop> {
  ControlLoop(const std::array<float, 6> &positions, const std::array<float, 6> &velocities)
      : current_positions(positions), current_velocities(velocities) {}
  std::array<float, 6> current_positions;
  std::array<float, 6> current_velocities;
};

/*******************************************/
struct ForceModeSwitch : etl::message<EventId::kForceModeSwitch> {
  explicit ForceModeSwitch(etl::fsm_state_id_t target) : target_mode(target) {}
  etl::fsm_state_id_t target_mode;
};

/*******************************************/
struct ExecutionReq : etl::message<EventId::kExecutionReq> {
  explicit ExecutionReq(JointTrajectory traj) : trajectory(std::move(traj)) {}
  JointTrajectory trajectory;
};

/*******************************************/
struct GoToPosReq : etl::message<EventId::kGoToPosReq> {
  explicit GoToPosReq(std::array<float, 6> positions) {
    point.positions.assign(positions.begin(), positions.end());
    point.velocities.assign(6, 0);
    point.accelerations.assign(6, 0);
    point.effort.assign(6, 0);
  }
  JointTrajectoryPoint point;
};

/*******************************************/
struct CancelExecutionReq : etl::message<EventId::kCancelExecutionReq> {};

/*******************************************/
struct GripperCommand : etl::message<EventId::kGripperCommand> {
  explicit GripperCommand(bool open) : target_position(open ? 0.453383446f : -1.11562538f) {}
  float target_position;
};

/*******************************************/
struct FaultClear : etl::message<EventId::kFaultClear> {};

/*******************************************/
struct OpticalSwitchTriggered : etl::message<EventId::kOpticalSwitchTriggered> {};

/*******************************************/
struct NextJoint : etl::message<EventId::kNextJoint> {};
}  // namespace event

class Arm : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = 0;

 public:
  Arm() : fsm(kMessageRouterId) {}

  bool planner_need_reset_{
      true};  ///< 从故障中恢复或刚上电的时候，规划器不知道当前关节位置/丢失跟踪了一段时间，需要在下一个ControlLoop到来的时候强制把规划器的当前位置重置为当前关节位置
};
extern Arm arm;

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
struct NoForce : etl::fsm_state<Arm, NoForce, StateId::kNoForce,  //
                                event::ForceModeSwitch,           //
                                event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    get_fsm_context().planner_need_reset_ = true;
    globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    globals->actuator.SetEnable(false, 1);
    globals->planner.Enable(false);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    globals->planner.ResetAt(e.current_positions);
    globals->planner.Update(e.current_positions, e.current_velocities, 0.002f);
    globals->actuator.SetEnable(false, 1);
    return No_State_Change;
  }
};

/*******************************************/
struct Manual : etl::fsm_state<Arm, Manual, StateId::kManual,  //
                               event::ForceModeSwitch,         //
                               event::ControlLoop,             //
                               event::NextJoint,               //
                               event::GripperCommand> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    target_positions_ = globals->planner.output().positions;
    globals->led_controller.SetPattern<rm::modules::led_pattern::RgbFlow>();
    globals->buzzer_controller.Play<buzzer_melody::EnterManualMode>();
    globals->planner.Enable(true);
    globals->actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    if (get_fsm_context().planner_need_reset_) {
      globals->planner.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    const auto &joint_limits = MotionPlanner::joint_limits();
    target_positions_[ctrl_joint_idx_] =
        std::clamp(target_positions_[ctrl_joint_idx_] + (globals->rc.left_x() * 0.001f),  //
                   joint_limits[ctrl_joint_idx_].first, joint_limits[ctrl_joint_idx_].second);
    const auto target_p = [this] {
      JointTrajectoryPoint p;
      p.positions.assign(target_positions_.begin(), target_positions_.end());
      p.velocities.resize(6);
      p.accelerations.resize(6);
      p.effort.resize(6);
      return p;
    }();
    globals->planner.GoTo(target_p);
    globals->planner.Update(e.current_positions, e.current_velocities, 0.002f);
    globals->actuator.ApplyOutput(globals->planner.output(), true);
    return No_State_Change;
  }
  REACT(event::NextJoint) {
    UNUSED(e);
    ++ctrl_joint_idx_;
    switch (ctrl_joint_idx_.get()) {
        // clang-format off
      case 0: globals->buzzer_controller.Play<buzzer_melody::HiBeeps<1>>(); break;
      case 1: globals->buzzer_controller.Play<buzzer_melody::HiBeeps<2>>(); break;
      case 2: globals->buzzer_controller.Play<buzzer_melody::HiBeeps<3>>(); break;
      case 3: globals->buzzer_controller.Play<buzzer_melody::HiBeeps<4>>(); break;
      case 4: globals->buzzer_controller.Play<buzzer_melody::HiBeeps<5>>(); break;
      case 5: globals->buzzer_controller.Play<buzzer_melody::HiBeeps<6>>(); break;
      default: break;
        // clang-format on
    }
    return No_State_Change;
  }
  REACT(event::GripperCommand) {
    globals->actuator.gripper->SetPosition(e.target_position, 0, 0, 10.f, 5.f);
    return No_State_Change;
  }

 private:
  std::array<float, 6> target_positions_{};
  etl::cyclic_value<int, 0, 6> ctrl_joint_idx_{0};
};

/*******************************************/
struct Idle : etl::fsm_state<Arm, Idle, StateId::kIdle,  //
                             event::ForceModeSwitch,     //
                             event::ExecutionReq,        //
                             event::ControlLoop,         //
                             event::GoToPosReq> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    globals->planner.Enable(true);
    globals->actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ExecutionReq) {
    if (globals->planner.status() == MotionPlanner::kIdle) {
      globals->planner.Execute(e.trajectory);
      return StateId::kExecuting;
    }
    return No_State_Change;
  }
  REACT(event::GoToPosReq) {
    if (globals->planner.status() == MotionPlanner::kIdle) {
      globals->planner.GoTo(e.point);
      return StateId::kExecuting;
    }
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    if (globals->planner.status() == MotionPlanner::kFault) {
      return StateId::kFault;
    }
    if (get_fsm_context().planner_need_reset_) {
      globals->planner.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }

    globals->planner.Update(e.current_positions, e.current_velocities, 0.002f);
    MotionPlanner::Output<6> out = MotionPlanner::Output<6>::Zero();
    out.efforts[1] = globals->planner.output().efforts[1];
    out.efforts[2] = globals->planner.output().efforts[2];
    out.positions[1] = globals->planner.output().positions[1];
    out.positions[2] = globals->planner.output().positions[2];
    globals->actuator.ApplyOutput(out, true);

    return No_State_Change;
  }
};

/*******************************************/
struct Executing : etl::fsm_state<Arm, Executing, StateId::kExecuting,  //
                                  event::ControlLoop,                   //
                                  event::ForceModeSwitch,               //
                                  event::CancelExecutionReq> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->led_controller.SetPattern<rm::modules::led_pattern::RgbFlow>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Success>();
    globals->planner.Enable(true);
    globals->actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::CancelExecutionReq) {
    UNUSED(e);
    const auto current_p = [&] {
      const auto &current_pos = globals->planner.output().positions;
      JointTrajectoryPoint p;
      p.positions.assign(current_pos.begin(), current_pos.end());
      p.velocities.resize(6);
      p.accelerations.resize(6);
      p.effort.resize(6);
      return p;
    }();
    globals->planner.GoTo(current_p);
    return No_State_Change;  // 机械臂停下来需要时间，所以不立刻转换到idle状态，而是等控制器状态变为idle时再转换
  }
  REACT(event::ControlLoop) {
    switch (globals->planner.status()) {
      case MotionPlanner::kIdle: {
        return StateId::kIdle;
      }
      case MotionPlanner::kFault: {
        return StateId::kFault;
      }
      case MotionPlanner::kExecuting:
      default: {
        // no state change
        break;
      }
    }
    if (get_fsm_context().planner_need_reset_) {
      globals->planner.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    globals->planner.Update(e.current_positions, e.current_velocities, 0.002f);
    MotionPlanner::Output<6> out = MotionPlanner::Output<6>::Zero();
    out.efforts[1] = globals->planner.output().efforts[1];
    out.efforts[2] = globals->planner.output().efforts[2];
    out.positions[1] = globals->planner.output().positions[1];
    out.positions[2] = globals->planner.output().positions[2];
    globals->actuator.ApplyOutput(out, true);
    return No_State_Change;
  }
};

/*******************************************/
struct J6Calibrating : etl::fsm_state<Arm, J6Calibrating, StateId::kJ6Calibrating,  //
                                      event::ControlLoop,                           //
                                      event::ForceModeSwitch,                       //
                                      event::OpticalSwitchTriggered> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER { return No_State_Change; }
  REACT(event::ControlLoop) {
    UNUSED(e);
    return No_State_Change;
  }
  REACT(event::OpticalSwitchTriggered) {
    UNUSED(e);
    return No_State_Change;
  }
};

/*******************************************/
struct Fault : etl::fsm_state<Arm, Fault, StateId::kFault,  //
                              event::ControlLoop,           //
                              event::FaultClear> {
  // ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    get_fsm_context().planner_need_reset_ = true;
    globals->actuator.SetEnable(false);
    globals->planner.Enable(false);
    globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    globals->actuator.SetEnable(false, 1);
    return No_State_Change;
  }
  REACT(event::FaultClear) {
    UNUSED(e);
    globals->planner.FaultClear();
    return StateId::kIdle;
  }
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm