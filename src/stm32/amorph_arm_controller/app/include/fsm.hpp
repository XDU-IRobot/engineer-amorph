
#pragma once

#include <librm.hpp>
#include <etl/fsm.h>

#include "arm_controller.hpp"
#include "rpc/host_api.hpp"

namespace fsm {

struct EventId {
  enum {
    kControlLoop,             ///< 控制循环时钟事件
    kForceModeSwitch,         ///< 切换模式命令
    kExecutionReq,            ///< 轨迹执行请求
    kGoToPosReq,              ///< 定点去往请求
    kCancelExecutionReq,      ///< 取消轨迹执行请求
    kFaultClear,              ///< 故障清除命令
    kOpticalSwitchTriggered,  ///< 光电开关触发，j6回零完成
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
  std::array<float, 6> current_positions;
  std::array<float, 6> current_velocities;
};

/*******************************************/
struct ForceModeSwitch : etl::message<EventId::kForceModeSwitch> {
  etl::fsm_state_id_t target_mode;
};

/*******************************************/
struct ExecutionReq : etl::message<EventId::kExecutionReq> {
  JointTrajectory trajectory;
};

/*******************************************/
struct GoToPosReq : etl::message<EventId::kGoToPosReq> {
  explicit GoToPosReq(std::array<float, 6> positions) { point.positions.assign(positions.begin(), positions.end()); }
  JointTrajectoryPoint point;
};

/*******************************************/
struct CancelExecutionReq : etl::message<EventId::kCancelExecutionReq> {};

/*******************************************/
struct FaultClear : etl::message<EventId::kFaultClear> {};

/*******************************************/
struct OpticalSwitchTriggered : etl::message<EventId::kOpticalSwitchTriggered> {};
}  // namespace event

class Arm : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = 0;

  using DmMit = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;
  using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

 public:
  Arm() : fsm(kMessageRouterId) {}

  void SetActuator(DmMit *j1, DmMit *j2, DmMit *j3, DmMit *j4, DmMit *j5, rm::device::M2006 *j6) {
    actuator_.j1 = j1;
    actuator_.j2 = j2;
    actuator_.j3 = j3;
    actuator_.j4 = j4;
    actuator_.j5 = j5;
    actuator_.j6 = j6;
  }

  ArmController controller_;
  struct Actuator {
    void SetEnable(bool enable) const {
      const auto cmd = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
      j1->SendInstruction(cmd);
      j2->SendInstruction(cmd);
      j3->SendInstruction(cmd);
      j4->SendInstruction(cmd);
      j5->SendInstruction(cmd);
    }
    void ApplyOutput(const ArmController::Output<6> &output) const {
      j1->SetPosition(0, 0, output.efforts[0], 0, 0);
      j2->SetPosition(0, 0, output.efforts[1], 0, 0);
      j3->SetPosition(0, 0, output.efforts[2], 0, 0);
      j4->SetPosition(0, 0, output.efforts[3], 0, 0);
      j5->SetPosition(0, 0, output.efforts[4], 0, 0);
      j6->SetCurrent(output.efforts[5]);
      rm::device::DjiMotor<>::SendCommand();
    }
    Actuator() = default;
    DmMit *j1, *j2, *j3, *j4, *j5;
    rm::device::M2006 *j6;
  } actuator_{};
};

namespace state {

#define ACCEPT_MODE_SWITCH() \
  etl::fsm_state_id_t on_event(const event::ForceModeSwitch &e) { return e.target_mode; }
#define IGNORE_UNINTEREST_EVENT() \
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &) { return No_State_Change; }
#define ENTER etl::fsm_state_id_t on_enter_state()
#define REACT(EventType) etl::fsm_state_id_t on_event(const EventType &e)

/*******************************************/
struct NoForce : etl::fsm_state<Arm, NoForce, StateId::kNoForce,  //
                                event::ForceModeSwitch,           //
                                event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &arm = get_fsm_context();
    arm.actuator_.SetEnable(false);
    arm.controller_.Enable(false);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &arm = get_fsm_context();
    arm.controller_.Update(e.current_positions, e.current_velocities);
    arm.actuator_.SetEnable(false);
    return No_State_Change;
  }
};
extern NoForce no_force;

/*******************************************/
struct Manual : etl::fsm_state<Arm, Manual, StateId::kManual,  //
                               event::ForceModeSwitch,         //
                               event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &arm = get_fsm_context();
    arm.controller_.Enable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &arm = get_fsm_context();
    arm.controller_.Update(e.current_positions, e.current_velocities);
    arm.actuator_.ApplyOutput(arm.controller_.output());
    return No_State_Change;
  }
};
extern Manual manual;

/*******************************************/
struct Idle : etl::fsm_state<Arm, Idle, StateId::kIdle,  //
                             event::ForceModeSwitch,     //
                             event::ExecutionReq> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &arm = get_fsm_context();
    arm.controller_.Enable(true);
    return No_State_Change;
  }
  REACT(event::ExecutionReq) {
    auto &arm = get_fsm_context();
    if (arm.controller_.status() == ArmController::kIdle) {
      arm.controller_.Execute(e.trajectory);
      return StateId::kExecuting;
    }
  }
  REACT(event::ControlLoop) {
    auto &arm = get_fsm_context();
    arm.controller_.Update(e.current_positions, e.current_velocities);
    arm.actuator_.ApplyOutput(arm.controller_.output());
    return No_State_Change;
  }
};
extern Idle idle;

/*******************************************/
struct Executing : etl::fsm_state<Arm, Executing, StateId::kExecuting,  //
                                  event::ForceModeSwitch,               //
                                  event::CancelExecutionReq> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER { return No_State_Change; }
  REACT(event::CancelExecutionReq) {
    auto &arm = get_fsm_context();
    const auto current_p = [&] {
      const auto &current_pos = arm.controller_.output().positions;
      JointTrajectoryPoint p;
      p.positions.assign(current_pos.begin(), current_pos.end());
      p.velocities.resize(6);
      p.accelerations.resize(6);
      p.effort.resize(6);
      return p;
    }();
    arm.controller_.GoTo(current_p);
    return No_State_Change;  // 机械臂停下来需要时间，所以不立刻转换到idle状态，而是等控制器状态变为idle时再转换
  }
  REACT(event::ControlLoop) {
    auto &arm = get_fsm_context();
    arm.controller_.Update(e.current_positions, e.current_velocities);
    arm.actuator_.ApplyOutput(arm.controller_.output());
    switch (arm.controller_.status()) {
      case ArmController::kIdle: {
        return StateId::kIdle;
      }
      case ArmController::kFault: {
        return StateId::kFault;
      }
      case ArmController::kExecuting:
      default: {
        return No_State_Change;
      }
    }
  }
};
extern Executing executing;

/*******************************************/
struct J6Calibrating : etl::fsm_state<Arm, J6Calibrating, StateId::kJ6Calibrating,  //
                                      event::ControlLoop,                           //
                                      event::ForceModeSwitch,                       //
                                      event::OpticalSwitchTriggered> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER { auto &arm = get_fsm_context(); }
  REACT(event::OpticalSwitchTriggered) {}
};
extern J6Calibrating j6_calibrating;

/*******************************************/
struct Fault : etl::fsm_state<Arm, Fault, StateId::kFault,  //
                              event::FaultClear> {
  IGNORE_UNINTEREST_EVENT();
  ENTER { return No_State_Change; }
  REACT(event::FaultClear) {
    UNUSED(e);
    return StateId::kIdle;
  }
};
extern Fault fault;

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

}  // namespace fsm