
#pragma once

#include <etl/fsm.h>

#include "arm_controller.hpp"

namespace fsm {

struct EventId {
  enum {
    kControlLoop,         ///< 控制循环时钟事件
    kForceModeSwitch,     ///< 切换模式命令
    kExecutionReq,        ///< 轨迹执行请求
    kGoToPosReq,          ///< 定点去往请求
    kCancelExecutionReq,  ///< 取消轨迹执行请求
    kFaultClear,          ///< 故障清除命令
  };
};

struct StateId {
  enum {
    kNoForce,    ///< 强制无力
    kManual,     ///< 遥控器手动模式
    kIdle,       ///< 空闲，可执行
    kExecuting,  ///< 正在执行运动
    kFault,      ///< 故障状态
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
  StateId target_mode;
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
}  // namespace event

class Arm : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = 0;

  using DmMit = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;
  using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

 public:
  Arm(DmMit *j1, DmMit *j2, DmMit *j3, DmMit *j4, DmMit *j5, rm::device::M2006 *j6)
      : fsm(kMessageRouterId),             //
        actuator_{j1, j2, j3, j4, j5, j6}  //
  {}

  ArmController controller_;
  struct Actuator {
    void SetEnableAll(bool enable) const {
      const auto instruction =
          enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
      j1->SendInstruction(instruction);
      j2->SendInstruction(instruction);
      j3->SendInstruction(instruction);
      j4->SendInstruction(instruction);
      j5->SendInstruction(instruction);
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
    DmMit *j1, *j2, *j3, *j4, *j5;
    rm::device::M2006 *j6;
  } actuator_;
};

namespace state {

#define IGNORE_UNINTEREST_EVENT() \
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &) { return No_State_Change; }
#define ENTER etl::fsm_state_id_t on_enter_state()
#define REACT(EventType) etl::fsm_state_id_t on_event(const EventType &e)

/*******************************************/
struct NoForce : etl::fsm_state<Arm, NoForce, StateId::kNoForce,  //
                                event::ControlLoop> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &arm = get_fsm_context();
    arm.actuator_.SetEnableAll(false);
    arm.controller_.Enable(false);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &arm = get_fsm_context();
    arm.controller_.Update(e.current_positions, e.current_velocities);
    arm.actuator_.SetEnableAll(false);
    return No_State_Change;
  }
};
extern NoForce no_force;

/*******************************************/
struct Manual : etl::fsm_state<Arm, Manual, StateId::kManual,  //
                               event::ControlLoop> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &arm = get_fsm_context();
    arm.controller_.Enable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    // TODO
    return No_State_Change;
  }
};
extern Manual manual;

/*******************************************/
struct Idle : etl::fsm_state<Arm, Idle, StateId::kIdle,  //
                             event::ExecutionReq> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &arm = get_fsm_context();
    arm.controller_.Enable(true);
    return No_State_Change;
  }
  REACT(event::ExecutionReq) {
    // TODO
    return StateId::kExecuting;
  }
};
extern Idle idle;

/*******************************************/
struct Executing : etl::fsm_state<Arm, Executing, StateId::kExecuting,  //
                                  event::CancelExecutionReq,            //
                                  event::ExecutionReq> {
  IGNORE_UNINTEREST_EVENT();
  ENTER { return No_State_Change; }
  REACT(event::CancelExecutionReq) {
    // TODO
    return StateId::kExecuting;
  }
  REACT(event::ExecutionReq) {
    // TODO
    return StateId::kExecuting;
  }
};
extern Executing executing;

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