
#pragma once

#include <etl/fsm.h>

#include "arm_controller.hpp"

namespace fsm {

constexpr inline etl::message_router_id_t kArmMRId = 0;
class Arm : public etl::fsm {
 public:
  Arm() : fsm(kArmMRId) {}

  void LogUnknownEvent(etl::imessage& msg) {}

 protected:
  ArmController arm_controller_;
};

namespace event {

struct EventId {
  enum {
    kExecutionRequest,        ///< 轨迹执行请求
    kCancelExecutionRequest,  ///< 取消轨迹执行请求
  };
};
class ExecutionRequest : public etl::message<EventId::kExecutionRequest> {
  JointTrajectory trajectory;
};
class CancelExecutionRequest : public etl::message<EventId::kCancelExecutionRequest> {};

}  // namespace event

namespace state {

struct StateId {
  enum {
    kNoForce,    ///< 强制无力
    kManual,     ///< 遥控器手动模式
    kIdle,       ///< 空闲，可执行轨迹
    kExecuting,  ///< 正在执行轨迹中
    kFault,      ///< 故障状态
  };
};

class Idle : public etl::fsm_state<Arm, Idle, StateId::kIdle, event::ExecutionRequest> {
 public:
  etl::fsm_state_id_t on_event(const event::ExecutionRequest& event) { return StateId::kExecuting; }
  etl::fsm_state_id_t on_event_unknown(const etl::imessage& event) { return No_State_Change; }
};

}  // namespace state

}  // namespace fsm