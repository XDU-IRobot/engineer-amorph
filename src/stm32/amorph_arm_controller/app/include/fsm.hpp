
#pragma once

#include <librm.hpp>
#include <etl/fsm.h>

#include "arm_controller.hpp"
#include "globals.hpp"

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
  explicit ExecutionReq(const JointTrajectory &traj) : trajectory(traj) {}
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

 public:
  Arm() : fsm(kMessageRouterId) {}
};
extern Arm arm;

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
    globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    globals->actuator.SetEnable(false, 1);
    globals->arm_controller.Enable(false);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    globals->arm_controller.Update(e.current_positions, e.current_velocities);
    globals->actuator.SetEnable(false, 1);
    return No_State_Change;
  }
};

/*******************************************/
struct Manual : etl::fsm_state<Arm, Manual, StateId::kManual,  //
                               event::ForceModeSwitch,         //
                               event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->arm_controller.Enable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    globals->arm_controller.Update(e.current_positions, e.current_velocities);
    // globals->actuator.ApplyOutput(globals->arm_controller.output());
    return No_State_Change;
  }
};

inline DynamicModel::Vec3 gcomp;

/*******************************************/
struct Idle : etl::fsm_state<Arm, Idle, StateId::kIdle,  //
                             event::ForceModeSwitch,     //
                             event::ExecutionReq,        //
                             event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    globals->arm_controller.Enable(true);
    globals->actuator.SetEnable(true);
    globals->arm_controller.GoTo({
        .positions = {0, 1.57f, -1.57f, 0, 0, 0},
        .velocities = {0, 0, 0, 0, 0, 0},
        .accelerations = {0, 0, 0, 0, 0, 0},
        .effort = {0, 0, 0, 0, 0, 0},
    });
    return No_State_Change;
  }
  REACT(event::ExecutionReq) {
    if (globals->arm_controller.status() == ArmController::kIdle) {
      globals->arm_controller.Execute(e.trajectory);
      return StateId::kExecuting;
    }
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    static DynamicModel dm;
    gcomp = dm.Inverse({e.current_positions[0], e.current_positions[1], e.current_positions[2]});
    globals->arm_controller.Update(e.current_positions, e.current_velocities, 0.001f);
    ArmController::Output<6> out = ArmController::Output<6>::Zero();
    out.efforts[1] = -gcomp(1);
    out.efforts[2] = gcomp(2);
    // out.positions[1] = globals->arm_controller.output().positions[1];
    // out.positions[2] = globals->arm_controller.output().positions[2];
    globals->actuator.ApplyOutput(out);
    // globals->actuator.ApplyOutput(ArmController::Output<6>::Zero());
    // if (globals->arm_controller.status() == ArmController::kFault) {
    //   return StateId::kFault;
    // }
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
  ENTER { return No_State_Change; }
  REACT(event::CancelExecutionReq) {
    const auto current_p = [&] {
      const auto &current_pos = globals->arm_controller.output().positions;
      JointTrajectoryPoint p;
      p.positions.assign(current_pos.begin(), current_pos.end());
      p.velocities.resize(6);
      p.accelerations.resize(6);
      p.effort.resize(6);
      return p;
    }();
    globals->arm_controller.GoTo(current_p);
    return No_State_Change;  // 机械臂停下来需要时间，所以不立刻转换到idle状态，而是等控制器状态变为idle时再转换
  }
  REACT(event::ControlLoop) {
    globals->arm_controller.Update(e.current_positions, e.current_velocities);
    // globals->actuator.ApplyOutput(globals->arm_controller.output());
    switch (globals->arm_controller.status()) {
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

/*******************************************/
struct J6Calibrating : etl::fsm_state<Arm, J6Calibrating, StateId::kJ6Calibrating,  //
                                      event::ControlLoop,                           //
                                      event::ForceModeSwitch,                       //
                                      event::OpticalSwitchTriggered> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER { return No_State_Change; }
  REACT(event::ControlLoop) { return No_State_Change; }
  REACT(event::OpticalSwitchTriggered) { return No_State_Change; }
};

/*******************************************/
struct Fault : etl::fsm_state<Arm, Fault, StateId::kFault,  //
                              event::ControlLoop,           //
                              event::FaultClear> {
  // ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->actuator.SetEnable(false);
    globals->arm_controller.Enable(false);
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) { return No_State_Change; }
  REACT(event::FaultClear) {
    UNUSED(e);
    globals->arm_controller.FaultClear();
    return StateId::kIdle;
  }
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm