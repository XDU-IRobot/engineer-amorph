
#pragma once

#include <etl/fsm.h>

namespace fsm {

constexpr inline etl::message_router_id_t kMotorControl = 0;
class Arm : public etl::fsm {
 public:
  Arm() : fsm(kMotorControl) {}

  void LogUnknownEvent(etl::imessage& msg) {}
};

namespace event {
struct EventId {
  enum { kExecuteTrajectory, kCancelExecution };
};
class ExecuteTrajectory : public etl::message<EventId::kExecuteTrajectory> {};
class CancelExecution : public etl::message<EventId::kCancelExecution> {};
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
}  // namespace state

}  // namespace fsm