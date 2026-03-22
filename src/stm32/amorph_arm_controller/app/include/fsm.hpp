
#pragma once

#include <librm.hpp>
#include <etl/fsm.h>

#include "host_comm.hpp"
#include "arm_controller.hpp"
#include "globals.hpp"
#include "kinematics.hpp"

namespace fsm {

struct EventId {
  enum {
    kControlLoop,                 ///< 控制循环时钟事件
    kForceModeSwitch,             ///< 切换模式命令
    kCustomCtrlCommand,           ///< 自定义控制器命令
    kCartesianCustomCtrlCommand,  ///< 笛卡尔空间自定义控制器命令（T265）
    kHostCommand,                 ///< 上位机下发的命令
    kFaultClear,                  ///< 故障清除命令
    kNextJoint,                   ///< Manual模式下切换到下一个关节
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kNoForce,               ///< 强制无力
    kManual,                ///< 遥控器手动模式，调试/检录的时候用
    kIdle,                  ///< 空闲，可执行轨迹
    kHostControl,           ///< 正在被上位机控制
    kCustomCtrl,            ///< 自定义控制器操作
    kCartesianCustomCtrl,   ///< 笛卡尔空间自定义控制器操作（T265）
    kLongJointCalibrating,  ///< 夹爪校准位置
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
struct CustomCtrlCommand : etl::message<EventId::kCustomCtrlCommand>, ::CustomControllerCommand {
  using CustomControllerCommand::CustomControllerCommand;
  CustomCtrlCommand() = default;
  explicit CustomCtrlCommand(const CustomControllerCommand &command) : CustomControllerCommand(command) {}
};

/*******************************************/
struct CartesianCustomCtrlCommand : etl::message<EventId::kCartesianCustomCtrlCommand>,
                                    ::CartesianCustomControllerCommand {
  using CartesianCustomControllerCommand::CartesianCustomControllerCommand;
  CartesianCustomCtrlCommand() = default;
  explicit CartesianCustomCtrlCommand(const CartesianCustomControllerCommand &command)
      : CartesianCustomControllerCommand(command) {}
};

/*******************************************/
struct HostCommand : etl::message<EventId::kHostCommand>, ::HostCommand {
  using ::HostCommand::HostCommand;
  HostCommand() = default;
  explicit HostCommand(const ::HostCommand &command) : ::HostCommand(command) {}
};

/*******************************************/
struct FaultClear : etl::message<EventId::kFaultClear> {};

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
    subloop_divide_10_ = 0;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    subloop_divide_10_ = (subloop_divide_10_ + 1) % 10;
    globals->arm_controller.ResetAt(e.current_positions);
    globals->arm_controller.Update(e.current_positions, e.current_velocities,
                                   globals->actuator.gripper.feedback().power, 0.002f);
    if (subloop_divide_10_ == 0) {
      globals->actuator.SetEnable(false, 1);
    }

    return No_State_Change;
  }

 private:
  int subloop_divide_10_{0};
};

/*******************************************/
struct Manual : etl::fsm_state<Arm, Manual, StateId::kManual,  //
                               event::ForceModeSwitch,         //
                               event::ControlLoop,             //
                               event::NextJoint> {
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
      target_positions_ = globals->arm_controller.current_positions();
      target_positions_initialized_ = true;
    }
    const auto &joint_limits = ArmController<7>::joint_limits();
    target_positions_[ctrl_joint_idx_] =
        std::clamp(target_positions_[ctrl_joint_idx_] + (globals->rc.left_x() * 0.001f),  //
                   joint_limits[ctrl_joint_idx_].first, joint_limits[ctrl_joint_idx_].second);
    target_positions_[6] = (globals->rc.ch(RcReceiver::kSf) < 1000) ? 1.f : 0.f;  // sf控制夹爪开关
    globals->arm_controller.GoTo(target_positions_);
    globals->arm_controller.Update(e.current_positions, e.current_velocities,
                                   globals->actuator.gripper.feedback().power, 0.002f);

    auto output = globals->arm_controller.output();
    globals->actuator.ApplyOutput(output, true);
    return No_State_Change;
  }
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

 private:
  std::array<float, 7> target_positions_{};
  int ctrl_joint_idx_{0};
  bool target_positions_initialized_{false};
};

/*******************************************/
struct Idle : etl::fsm_state<Arm, Idle, StateId::kIdle,  //
                             event::ForceModeSwitch,     //
                             event::ControlLoop,         //
                             event::HostCommand,         //
                             event::CustomCtrlCommand,   //
                             event::CartesianCustomCtrlCommand> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    get_fsm_context().planner_need_reset_ = true;  // 进入Idle时强制重置规划器，防止位置跳变
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    globals->actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    // if (globals->arm_controller.status() == ArmController<7>::kFault) {
    //   return StateId::kFault;
    // }
    if (get_fsm_context().planner_need_reset_) {
      globals->arm_controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }

    globals->arm_controller.Update(e.current_positions, e.current_velocities,
                                   globals->actuator.gripper.feedback().power, 0.002f);
    globals->actuator.ApplyOutput(globals->arm_controller.output(), true);
    return No_State_Change;
  }
  REACT(event::HostCommand) {
    if (e.take_control) {
      return StateId::kHostControl;
    }
    return No_State_Change;
  }
  REACT(event::CustomCtrlCommand) {
    if (e.buttons.a) {  // 按下按钮a，自定义控制器接管
      return StateId::kCustomCtrl;
    }
    return No_State_Change;
  }
  REACT(event::CartesianCustomCtrlCommand) {
    if (e.btn_a) {  // 按下按钮a，自定义控制器接管
      return StateId::kCartesianCustomCtrl;
    }
    return No_State_Change;
  }
};

/*******************************************/
struct HostControl : etl::fsm_state<Arm, HostControl, StateId::kHostControl,  //
                                    event::ControlLoop,                       //
                                    event::ForceModeSwitch,                   //
                                    event::HostCommand> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Success>();
    globals->actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    switch (globals->arm_controller.status()) {
      // case ArmController<7>::kFault: {
      //   return StateId::kFault;
      // }
      case ArmController<7>::kMoving:
      default: {
        // no state change
        break;
      }
    }
    if (get_fsm_context().planner_need_reset_) {
      globals->arm_controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    globals->arm_controller.Update(e.current_positions, e.current_velocities,
                                   globals->actuator.gripper.feedback().power, 0.002f);
    globals->actuator.ApplyOutput(globals->arm_controller.output(), true);
    return No_State_Change;
  }
  REACT(event::HostCommand) {
    if (!e.take_control) {
      return StateId::kIdle;
    }
    std::array<float, 7> target_p = globals->arm_controller.current_positions();
    target_p[0] = e.target_joint_positions[0];
    target_p[1] = e.target_joint_positions[1];
    target_p[2] = e.target_joint_positions[2];
    target_p[3] = e.target_joint_positions[3];
    target_p[4] = e.target_joint_positions[4];
    target_p[5] = e.target_joint_positions[5];
    target_p[6] = e.gripper_open ? 1.f : 0.f;
    globals->arm_controller.GoTo(target_p);
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
    get_fsm_context().planner_need_reset_ = true;  // 强制重置规划器，防止位置跳变
    globals->buzzer_controller.Play<buzzer_melody::HiBeeps<1>>();
    globals->actuator.SetEnable(true);
    just_entered_ = true;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    // if (globals->arm_controller.status() == ArmController<7>::kFault) {
    //   return StateId::kFault;
    // }
    if (get_fsm_context().planner_need_reset_) {
      globals->arm_controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    globals->arm_controller.Update(e.current_positions, e.current_velocities,
                                   globals->actuator.gripper.feedback().power, 0.002f);
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
    if (just_entered_) {
      j6_delta_ = e.joint_positions[5] - globals->arm_controller.current_positions()[5];
      just_entered_ = false;
    }
    std::array<float, 7> target_p = globals->arm_controller.current_positions();
    target_p[0] = e.joint_positions[0];
    target_p[1] = e.joint_positions[1];
    target_p[2] = e.joint_positions[2];
    target_p[3] = e.joint_positions[3];
    target_p[4] = e.joint_positions[4];
    target_p[5] = e.joint_positions[5] - j6_delta_;
    target_p[6] = e.buttons.b ? 0.f : 1.f;
    globals->arm_controller.GoTo(target_p);
    return No_State_Change;
  }

 private:
  bool just_entered_{true};
  float j6_delta_{0.f};
};

/*******************************************/
struct CartesianCustomCtrl : etl::fsm_state<Arm, CartesianCustomCtrl, StateId::kCartesianCustomCtrl,  //
                                            event::ControlLoop,                                       //
                                            event::ForceModeSwitch,                                   //
                                            event::CartesianCustomCtrlCommand> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    get_fsm_context().planner_need_reset_ = true;  // 强制重置规划器，防止位置跳变
    globals->buzzer_controller.Play<buzzer_melody::HiBeeps<1>>();
    globals->actuator.SetEnable(true);
    // 进入笛卡尔空间自定义控制模式时，记录当前末端位置作为初始位置，后续的控制命令都是相对于这个初始位置的增量
    Eigen::Matrix3f dummy;
    const auto &current_p = globals->actuator.GetPositions();
    k_.SolveFk({current_p[0], current_p[1], current_p[2], current_p[3], current_p[4], current_p[5]},
               end_eff_initial_pos_, dummy);
    just_entered_ = true;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    // if (globals->arm_controller.status() == ArmController<7>::kFault) {
    //   return StateId::kFault;
    // }
    if (get_fsm_context().planner_need_reset_) {
      globals->arm_controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    globals->arm_controller.Update(e.current_positions, e.current_velocities,
                                   globals->actuator.gripper.feedback().power, 0.002f);
    globals->actuator.ApplyOutput(globals->arm_controller.output(), true);
    return No_State_Change;
  }
  REACT(event::CartesianCustomCtrlCommand) {
    if (!e.btn_a) {  // 松开按钮a，退出CustomCtrl模式
      return StateId::kIdle;
    }
    // if (!e.ok) {
    //   return No_State_Change;
    // }
    if (just_entered_) {
      cc_initial_pos_ = {e.translation_xyz[0], e.translation_xyz[1], e.translation_xyz[2]};
      just_entered_ = false;
    }
    const Eigen::Quaternionf target_end_eff_rotation{e.rotation_wxyz[0], e.rotation_wxyz[1], e.rotation_wxyz[2],
                                                     e.rotation_wxyz[3]};
    const auto cc_pos_delta =
        Eigen::Vector3f{e.translation_xyz[0], e.translation_xyz[1], e.translation_xyz[2]} - cc_initial_pos_;
    const std::array current_p_6 = {
        globals->arm_controller.current_positions()[0], globals->arm_controller.current_positions()[1],
        globals->arm_controller.current_positions()[2], globals->arm_controller.current_positions()[3],
        globals->arm_controller.current_positions()[4], globals->arm_controller.current_positions()[5]};
    const auto target_p_6 =
        k_.SolveIk(end_eff_initial_pos_ + cc_pos_delta, target_end_eff_rotation.toRotationMatrix(), current_p_6);
    if (!target_p_6.has_value()) {
      // IK求解失败，可能是因为目标位置不可达，直接忽略这条命令
      return No_State_Change;
    }
    target_p_[0] = target_p_6.value()[0];
    target_p_[1] = target_p_6.value()[1];
    target_p_[2] = target_p_6.value()[2];
    target_p_[3] = target_p_6.value()[3];
    target_p_[4] = target_p_6.value()[4];
    target_p_[5] = target_p_6.value()[5];
    target_p_[6] = e.btn_b ? 0.f : 1.f;
    globals->arm_controller.GoTo(target_p_);
    return No_State_Change;
  }

 private:
  bool just_entered_{true};
  Eigen::Vector3f end_eff_initial_pos_;
  Eigen::Vector3f cc_initial_pos_;
  std::array<float, 7> target_p_{};
  Kinematics k_;
};

/*******************************************/
struct LongJointCalibrating : etl::fsm_state<Arm, LongJointCalibrating, StateId::kLongJointCalibrating,  //
                                             event::ControlLoop,                                         //
                                             event::ForceModeSwitch> {
  // ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    stall_count_ = 0;
    calib_start_time_ = HAL_GetTick();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    globals->actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ForceModeSwitch) {
    if (e.target_mode == StateId::kNoForce) {
      return StateId::kNoForce;
    }
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    const uint32_t calib_elapsed = HAL_GetTick() - calib_start_time_;
    if (calib_elapsed > 2000) {
      // 长时间没有检测到堵转，认为校准失败，进入故障状态
      return StateId::kFault;
    }
    globals->actuator.gripper.SetSpeed(2.f * M_PI * 30.f);
    if (globals->actuator.gripper.feedback().speed_rad_ps < 50.f &&  // 转速低
        globals->actuator.gripper.feedback().power > 690 &&          // 力矩大
        calib_elapsed > 250                                          // 过了初始加速的阶段
    ) {
      // 认为电机堵转，增加一次堵转计数
      ++stall_count_;
    } else {
      stall_count_ = 0;
    }
    if (stall_count_ > 20) {  // 连续多次堵转，认为校准完成，更新编码器零点，并进入手动模式
      globals->actuator.gripper_encoder.Reset();
      globals->actuator.gripper_encoder.Update(globals->actuator.gripper.feedback().position_rad);
      globals->actuator.calibrators[6].UpdateConfig({false, globals->actuator.gripper_encoder.RealPosition()});
      return StateId::kManual;
    }
    return No_State_Change;
  }
  size_t stall_count_{0};
  uint32_t calib_start_time_{0};
};

/*******************************************/
struct Fault : etl::fsm_state<Arm, Fault, StateId::kFault,  //
                              event::ForceModeSwitch,       //
                              event::ControlLoop,           //
                              event::FaultClear> {
  // ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    get_fsm_context().planner_need_reset_ = true;
    globals->actuator.SetEnable(false);
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
    return No_State_Change;
  }
  REACT(event::ForceModeSwitch) {
    if (e.target_mode == StateId::kNoForce) {
      return StateId::kNoForce;
    }
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
    return StateId::kNoForce;
  }
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm