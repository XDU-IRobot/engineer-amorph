
#pragma once

#include <librm.hpp>
#include <etl/fsm.h>

#include "rpc/custom_controller.hpp"
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
    kGripperCommand,          ///< 夹爪控制命令
    kCustomCtrlCommand,       ///< 自定义控制器命令
    kFaultClear,              ///< 故障清除命令
    kOpticalSwitchTriggered,  ///< 光电开关触发，j6回零完成
    kNextJoint,               ///< Manual模式下切换到下一个关节
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kNoForce,               ///< 强制无力
    kManual,                ///< 遥控器手动模式，调试/检录的时候用
    kIdle,                  ///< 空闲，可执行轨迹
    kExecuting,             ///< 正在运动
    kCustomCtrl,            ///< 自定义控制器操作
    kLongJointCalibrating,  ///< 超过位置编码器1圈的长关节校准位置(j6、夹爪）
    kFault,                 ///< 故障状态
  };
};

namespace event {
/*******************************************/
struct ControlLoop : etl::message<EventId::kControlLoop> {
  ControlLoop(const std::array<float, 7> &positions,  //
              const std::array<float, 7> &velocities)
      : current_positions(positions), current_velocities(velocities) {}
  std::array<float, 7> current_positions;
  std::array<float, 7> current_velocities;
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
struct CustomCtrlCommand : etl::message<EventId::kCustomCtrlCommand>, CustomControllerCommand {
  using CustomControllerCommand::CustomControllerCommand;
  CustomCtrlCommand() = default;
  explicit CustomCtrlCommand(const CustomControllerCommand &command) : CustomControllerCommand(command) {}
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
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    globals->actuator.SetEnable(false, 1);
    globals->arm_controller.Enable(false);
    dji_motor_brake_count_ = 0;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    // if (dji_motor_brake_count_ < 10) {
    //   auto &j6_speed_pid = globals->arm_controller.pid().at(5);
    //   j6_speed_pid.Update(0, e.current_velocities[5]);
    //   globals->actuator.j6->SetCurrent(j6_speed_pid.out());
    //   rm::device::DjiMotorBase::SendCommand();
    //   ++dji_motor_brake_count_;
    // }
    globals->arm_controller.ResetAt(e.current_positions);
    globals->arm_controller.Update(e.current_positions, e.current_velocities, 0.002f);
    globals->actuator.SetEnable(false, 1);
    globals->actuator.j6.SetCurrent(0);
    globals->actuator.gripper.SetCurrent(0);
    rm::device::DjiMotorBase::SendCommand();
    return No_State_Change;
  }

 private:
  int dji_motor_brake_count_{0};
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
    ctrl_joint_idx_ = 0;
    // 进入Manual时强制重置规划器，防止位置跳变
    get_fsm_context().planner_need_reset_ = true;
    // 注意: target_positions_ 的初始化移动到第一个 ControlLoop 中进行，
    // 因为此时 planner_need_reset_ 为 true，arm_controller.output() 不是最新位置
    target_positions_initialized_ = false;
    globals->buzzer_controller.Play<buzzer_melody::EnterManualMode>();
    globals->arm_controller.Enable(true);
    globals->actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    if (get_fsm_context().planner_need_reset_) {
      globals->arm_controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    // 确保在 ResetAt 之后再初始化 target_positions_，避免位置跳变
    if (!target_positions_initialized_) {
      target_positions_ = globals->arm_controller.output().positions;
      target_positions_initialized_ = true;
    }
    const auto &joint_limits = ArmController<7>::joint_limits();
    target_positions_[ctrl_joint_idx_] =
        std::clamp(target_positions_[ctrl_joint_idx_] + (globals->rc.left_x() * 0.001f),  //
                   joint_limits[ctrl_joint_idx_].first, joint_limits[ctrl_joint_idx_].second);
    const auto target_p = [this] {
      JointTrajectoryPoint p;
      p.positions.assign(target_positions_.begin(), target_positions_.end());
      p.velocities.resize(7);
      p.accelerations.resize(7);
      p.effort.resize(7);
      return p;
    }();
    globals->arm_controller.GoTo(target_p);
    globals->arm_controller.Update(e.current_positions, e.current_velocities, 0.002f);

    // speed_eee.Update(globals->rc.left_x() * 60.f, e.gripper_velocity);
    // globals->actuator.gripper->SetCurrent(globals->rc.left_x() * 5000.f);
    // rm::device::DjiMotorBase::SendCommand();

    globals->actuator.ApplyOutput(globals->arm_controller.output(), true);
    return No_State_Change;
  }
  rm::modules::PID speed_eee{10.f, 0.f, 0.f, 6000.f, 0.f};
  REACT(event::NextJoint) {
    UNUSED(e);
    ctrl_joint_idx_ = (ctrl_joint_idx_ + 1) % 6;
    switch (ctrl_joint_idx_) {
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
    // globals->actuator.gripper->SetPosition(e.target_position, 0, 0, 10.f, 5.f);
    return No_State_Change;
  }

 private:
  std::array<float, 7> target_positions_{};
  int ctrl_joint_idx_{0};
  bool target_positions_initialized_{false};
};

/*******************************************/
struct Idle : etl::fsm_state<Arm, Idle, StateId::kIdle,  //
                             event::ForceModeSwitch,     //
                             event::ExecutionReq,        //
                             event::ControlLoop,         //
                             event::GoToPosReq,          //
                             event::CustomCtrlCommand> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    get_fsm_context().planner_need_reset_ = true;  // 进入Idle时强制重置规划器，防止位置跳变
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    globals->arm_controller.Enable(true);
    globals->actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ExecutionReq) {
    if (globals->arm_controller.status() == ArmController<7>::kIdle) {
      // 如果规划器需要重置，先不执行，等待下一个ControlLoop重置后再处理
      // 这种情况通常不会发生，因为ExecutionReq是异步事件，ControlLoop频率很高
      if (get_fsm_context().planner_need_reset_) {
        return No_State_Change;  // 暂不处理，等ControlLoop重置后再接受轨迹
      }
      globals->arm_controller.Execute(e.trajectory);
      return StateId::kExecuting;
    }
    return No_State_Change;
  }
  REACT(event::GoToPosReq) {
    if (globals->arm_controller.status() == ArmController<7>::kIdle) {
      // 如果规划器需要重置，先不执行，等待下一个ControlLoop重置后再处理
      if (get_fsm_context().planner_need_reset_) {
        return No_State_Change;  // 暂不处理，等ControlLoop重置后再接受目标点
      }
      globals->arm_controller.GoTo(e.point);
      return StateId::kExecuting;
    }
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    if (globals->arm_controller.status() == ArmController<7>::kFault) {
      return StateId::kFault;
    }
    if (get_fsm_context().planner_need_reset_) {
      globals->arm_controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }

    globals->arm_controller.Update(e.current_positions, e.current_velocities, 0.002f);
    globals->actuator.ApplyOutput(globals->arm_controller.output(), true);
    return No_State_Change;
  }
  REACT(event::CustomCtrlCommand) {
    if (e.buttons.a) {  // 按下按钮a，进入CustomCtrl模式
      return StateId::kCustomCtrl;
    }
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
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Success>();
    globals->arm_controller.Enable(true);
    globals->actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::CancelExecutionReq) {
    UNUSED(e);
    const auto current_p = [&] {
      const auto &current_pos = globals->arm_controller.output().positions;
      JointTrajectoryPoint p;
      p.positions.assign(current_pos.begin(), current_pos.end());
      p.velocities.resize(7);
      p.accelerations.resize(7);
      p.effort.resize(7);
      return p;
    }();
    globals->arm_controller.GoTo(current_p);
    return No_State_Change;  // 机械臂停下来需要时间，所以不立刻转换到idle状态，而是等控制器状态变为idle时再转换
  }
  REACT(event::ControlLoop) {
    switch (globals->arm_controller.status()) {
      case ArmController<7>::kIdle: {
        return StateId::kIdle;
      }
      case ArmController<7>::kFault: {
        return StateId::kFault;
      }
      case ArmController<7>::kExecuting:
      default: {
        // no state change
        break;
      }
    }
    if (get_fsm_context().planner_need_reset_) {
      globals->arm_controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    globals->arm_controller.Update(e.current_positions, e.current_velocities, 0.002f);
    globals->actuator.ApplyOutput(globals->arm_controller.output(), true);
    return No_State_Change;
  }
};

/*******************************************/
struct CustomCtrl : etl::fsm_state<Arm, CustomCtrl, StateId::kCustomCtrl,  //
                                   event::ControlLoop,                     //
                                   event::ForceModeSwitch,                 //
                                   event::CustomCtrlCommand> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    get_fsm_context().planner_need_reset_ = true;  // 进入CustomCtrl时强制重置规划器，防止位置跳变
    globals->buzzer_controller.Play<buzzer_melody::HiBeeps<1>>();
    globals->arm_controller.Enable(true);
    globals->actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    if (globals->arm_controller.status() == ArmController<7>::kFault) {
      return StateId::kFault;
    }
    if (get_fsm_context().planner_need_reset_) {
      globals->arm_controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    globals->arm_controller.Update(e.current_positions, e.current_velocities, 0.002f);
    globals->actuator.ApplyOutput(globals->arm_controller.output(), true);
    return No_State_Change;
  }
  REACT(event::CustomCtrlCommand) {
    if (!e.buttons.a) {  // 松开按钮a，退出CustomCtrl模式
      return StateId::kIdle;
    }
    if (!e.ok) {
      return No_State_Change;
    }
    auto target_p = [&] {
      const auto &current_pos = globals->arm_controller.output().positions;
      JointTrajectoryPoint p;
      p.positions.assign(current_pos.begin(), current_pos.end());
      p.velocities.resize(7);
      p.accelerations.resize(7);
      p.effort.resize(7);
      return p;
    }();
    target_p.positions[0] = e.joint_positions[0];
    target_p.positions[1] = e.joint_positions[1];
    target_p.positions[2] = e.joint_positions[2];
    target_p.positions[3] = e.joint_positions[3];
    target_p.positions[4] = e.joint_positions[4];
    globals->arm_controller.GoTo(target_p);
    return No_State_Change;
  }
};

/*******************************************/
struct LongJointCalibrating : etl::fsm_state<Arm, LongJointCalibrating, StateId::kLongJointCalibrating,  //
                                             event::ControlLoop,                                         //
                                             event::ForceModeSwitch,                                     //
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
    globals->arm_controller.Enable(false);
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