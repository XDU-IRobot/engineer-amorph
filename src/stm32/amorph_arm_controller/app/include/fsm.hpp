
#pragma once

#include <librm.hpp>
#include <etl/fsm.h>

#include "host_comm.hpp"
#include "arm_controller.hpp"
#include "actuators.hpp"
#include "kinematics.hpp"
#include "shared_resources.hpp"

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
struct ControlLoop : etl::message<EventId::kControlLoop> {};

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

/**
 * @brief FSM 状态访问其所属臂和共享资源的轻量上下文
 *
 * 使用裸指针（非所有权）以避免与 ArmInstance 产生循环包含。
 * ArmInstance 构造完成后立即调用 Arm::SetArmData 填入指针。
 */
struct ArmData {
  ArmController<7>* arm_controller{nullptr};  ///< 此臂的轨迹规划与控制器
  Actuator*         actuator{nullptr};        ///< 此臂的所有电机
  SharedResources*  shared{nullptr};          ///< 全局共享资源（蜂鸣器、LED、RC 等）
};

class Arm : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = 0;

 public:
  Arm() : fsm(kMessageRouterId) {}

  void SetArmData(const ArmData& data) { arm_data_ = data; }

  ArmController<7>& arm_controller() { return *arm_data_.arm_controller; }
  Actuator&         actuator()        { return *arm_data_.actuator; }
  SharedResources&  shared()          { return *arm_data_.shared; }

  /// IMU 加速度（机械臂坐标系，m/s²）—— 从共享资源直接读
  Eigen::Vector3f imu_acc() const {
    return {arm_data_.shared->imu.accel_x(),
            arm_data_.shared->imu.accel_y(),
            arm_data_.shared->imu.accel_z()};
  }

  bool planner_need_reset_{
      true};  ///< 从故障中恢复或刚上电的时候，规划器需要在下一个ControlLoop到来时强制重置为当前关节位置

 private:
  ArmData arm_data_{};
};

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
    get_fsm_context().shared().buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    get_fsm_context().actuator().SetEnable(false, 1);
    subloop_divide_10_ = 0;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    subloop_divide_10_ = (subloop_divide_10_ + 1) % 10;
    auto& actuator  = get_fsm_context().actuator();
    auto& arm_ctrl  = get_fsm_context().arm_controller();
    const auto pos  = actuator.GetPositions();
    const auto vel  = actuator.GetVelocities();
    arm_ctrl.ResetAt(pos);
    arm_ctrl.Update(pos, vel, actuator.gripper.feedback().power,
                    get_fsm_context().imu_acc(), 0.002f);
    if (subloop_divide_10_ == 0) {
      actuator.SetEnable(false, 1);
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
    target_positions_initialized_ = false;
    get_fsm_context().shared().buzzer_controller.Play<buzzer_melody::EnterManualMode>();
    get_fsm_context().actuator().SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    auto& arm_ctrl = get_fsm_context().arm_controller();
    auto& actuator = get_fsm_context().actuator();
    auto& shared   = get_fsm_context().shared();
    const auto pos = actuator.GetPositions();
    const auto vel = actuator.GetVelocities();

    if (get_fsm_context().planner_need_reset_) {
      arm_ctrl.ResetAt(pos);
      get_fsm_context().planner_need_reset_ = false;
    }
    if (!target_positions_initialized_) {
      target_positions_ = arm_ctrl.current_positions();
      target_positions_initialized_ = true;
    }
    const auto &joint_limits = arm_ctrl.joint_limits();
    target_positions_[ctrl_joint_idx_] =
        std::clamp(target_positions_[ctrl_joint_idx_] + (shared.rc.left_x() * 0.001f),
                   joint_limits[ctrl_joint_idx_].first, joint_limits[ctrl_joint_idx_].second);
    target_positions_[6] = (shared.rc.ch(RcReceiver::kSf) < 1000) ? 1.f : 0.f;
    arm_ctrl.GoTo(target_positions_);
    arm_ctrl.Update(pos, vel, actuator.gripper.feedback().power,
                    get_fsm_context().imu_acc(), 0.002f);
    actuator.ApplyOutput(arm_ctrl.output(), true);
    return No_State_Change;
  }
  REACT(event::NextJoint) {
    UNUSED(e);
    ctrl_joint_idx_ = (ctrl_joint_idx_ + 1) % 6;
    switch (ctrl_joint_idx_) {
        // clang-format off
      case 0: get_fsm_context().shared().buzzer_controller.Play<buzzer_melody::HiBeeps<1>>(); break;
      case 1: get_fsm_context().shared().buzzer_controller.Play<buzzer_melody::HiBeeps<2>>(); break;
      case 2: get_fsm_context().shared().buzzer_controller.Play<buzzer_melody::HiBeeps<3>>(); break;
      case 3: get_fsm_context().shared().buzzer_controller.Play<buzzer_melody::HiBeeps<4>>(); break;
      case 4: get_fsm_context().shared().buzzer_controller.Play<buzzer_melody::HiBeeps<5>>(); break;
      case 5: get_fsm_context().shared().buzzer_controller.Play<buzzer_melody::HiBeeps<6>>(); break;
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
    get_fsm_context().shared().buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    get_fsm_context().actuator().SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    auto& arm_ctrl = get_fsm_context().arm_controller();
    auto& actuator = get_fsm_context().actuator();
    const auto pos = actuator.GetPositions();
    const auto vel = actuator.GetVelocities();

    if (get_fsm_context().planner_need_reset_) {
      arm_ctrl.ResetAt(pos);
      get_fsm_context().planner_need_reset_ = false;
    }
    arm_ctrl.Update(pos, vel, actuator.gripper.feedback().power,
                    get_fsm_context().imu_acc(), 0.002f);
    actuator.ApplyOutput(arm_ctrl.output(), true);
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
    if (e.btn_b) {  // 没有按A使能自定义控制器的情况下，按B使机械臂归位
      get_fsm_context().shared().buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
      get_fsm_context().arm_controller().GoTo({0.f, 2.8f, -2.74f, 0.f, 0.f, 0.f, 1.f});
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
    get_fsm_context().shared().buzzer_controller.Play<rm::modules::buzzer_melody::Success>();
    get_fsm_context().actuator().SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    auto& arm_ctrl = get_fsm_context().arm_controller();
    auto& actuator = get_fsm_context().actuator();
    const auto pos = actuator.GetPositions();
    const auto vel = actuator.GetVelocities();

    switch (arm_ctrl.status()) {
      case ArmController<7>::kMoving:
      default:
        break;
    }
    if (get_fsm_context().planner_need_reset_) {
      arm_ctrl.ResetAt(pos);
      get_fsm_context().planner_need_reset_ = false;
    }
    arm_ctrl.Update(pos, vel, actuator.gripper.feedback().power,
                    get_fsm_context().imu_acc(), 0.002f);
    actuator.ApplyOutput(arm_ctrl.output(), true);
    return No_State_Change;
  }
  REACT(event::HostCommand) {
    if (!e.take_control) {
      return StateId::kIdle;
    }
    auto& arm_ctrl = get_fsm_context().arm_controller();
    std::array<float, 7> target_p = arm_ctrl.current_positions();
    target_p[0] = e.target_joint_positions[0];
    target_p[1] = e.target_joint_positions[1];
    target_p[2] = e.target_joint_positions[2];
    target_p[3] = e.target_joint_positions[3];
    target_p[4] = e.target_joint_positions[4];
    target_p[5] = e.target_joint_positions[5];
    target_p[6] = e.gripper_open ? 1.f : 0.f;
    arm_ctrl.GoTo(target_p);
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
    get_fsm_context().shared().buzzer_controller.Play<buzzer_melody::HiBeeps<1>>();
    get_fsm_context().actuator().SetEnable(true);
    just_entered_ = true;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    auto& arm_ctrl = get_fsm_context().arm_controller();
    auto& actuator = get_fsm_context().actuator();
    const auto pos = actuator.GetPositions();
    const auto vel = actuator.GetVelocities();

    if (get_fsm_context().planner_need_reset_) {
      arm_ctrl.ResetAt(pos);
      get_fsm_context().planner_need_reset_ = false;
    }
    arm_ctrl.Update(pos, vel, actuator.gripper.feedback().power,
                    get_fsm_context().imu_acc(), 0.002f);
    actuator.ApplyOutput(arm_ctrl.output(), true);
    return No_State_Change;
  }
  REACT(event::CustomCtrlCommand) {
    if (!e.buttons.a) {  // 松开按钮a，退出CustomCtrl模式
      return StateId::kIdle;
    }
    if (!e.ok) {
      return No_State_Change;
    }
    auto& arm_ctrl = get_fsm_context().arm_controller();
    if (just_entered_) {
      j6_delta_ = e.joint_positions[5] - arm_ctrl.current_positions()[5];
      just_entered_ = false;
    }
    std::array<float, 7> target_p = arm_ctrl.current_positions();
    target_p[0] = e.joint_positions[0];
    target_p[1] = e.joint_positions[1];
    target_p[2] = e.joint_positions[2];
    target_p[3] = e.joint_positions[3];
    target_p[4] = e.joint_positions[4];
    target_p[5] = e.joint_positions[5] - j6_delta_;
    target_p[6] = e.buttons.b ? 0.f : 1.f;
    arm_ctrl.GoTo(target_p);
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
    get_fsm_context().shared().buzzer_controller.Play<buzzer_melody::HiBeeps<1>>();
    get_fsm_context().actuator().SetEnable(true);
    // 进入笛卡尔空间自定义控制模式时，记录当前末端位置作为初始位置
    Eigen::Matrix3f dummy;
    const auto& current_p = get_fsm_context().actuator().GetPositions();
    k_.SolveFk({current_p[0], current_p[1], current_p[2], current_p[3], current_p[4], current_p[5]},
               end_eff_initial_pos_, dummy);
    just_entered_ = true;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    auto& arm_ctrl = get_fsm_context().arm_controller();
    auto& actuator = get_fsm_context().actuator();
    const auto pos = actuator.GetPositions();
    const auto vel = actuator.GetVelocities();

    if (get_fsm_context().planner_need_reset_) {
      arm_ctrl.ResetAt(pos);
      get_fsm_context().planner_need_reset_ = false;
    }
    arm_ctrl.Update(pos, vel, actuator.gripper.feedback().power,
                    get_fsm_context().imu_acc(), 0.002f);
    actuator.ApplyOutput(arm_ctrl.output(), true);
    return No_State_Change;
  }
  REACT(event::CartesianCustomCtrlCommand) {
    if (!e.btn_a) {  // 松开按钮a，退出CustomCtrl模式
      return StateId::kIdle;
    }
    auto& arm_ctrl = get_fsm_context().arm_controller();
    if (just_entered_) {
      cc_initial_pos_ = {e.translation_xyz[0], e.translation_xyz[1], e.translation_xyz[2]};
      just_entered_ = false;
    }
    const Eigen::Quaternionf target_end_eff_rotation{e.rotation_wxyz[0], e.rotation_wxyz[1], e.rotation_wxyz[2],
                                                     e.rotation_wxyz[3]};
    const auto cc_pos_delta =
        Eigen::Vector3f{e.translation_xyz[0], e.translation_xyz[1], e.translation_xyz[2]} - cc_initial_pos_;
    const std::array current_p_6 = {
        arm_ctrl.current_positions()[0], arm_ctrl.current_positions()[1],
        arm_ctrl.current_positions()[2], arm_ctrl.current_positions()[3],
        arm_ctrl.current_positions()[4], arm_ctrl.current_positions()[5]};
    const auto target_p_6 =
        k_.SolveIk(end_eff_initial_pos_ + cc_pos_delta, target_end_eff_rotation.toRotationMatrix(), current_p_6);
    if (!target_p_6.has_value()) {
      return No_State_Change;
    }
    target_p_[0] = target_p_6.value()[0];
    target_p_[1] = target_p_6.value()[1];
    target_p_[2] = target_p_6.value()[2];
    target_p_[3] = target_p_6.value()[3];
    target_p_[4] = target_p_6.value()[4];
    target_p_[5] = target_p_6.value()[5];
    target_p_[6] = e.btn_b ? 0.f : 1.f;
    arm_ctrl.GoTo(target_p_);
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
    get_fsm_context().shared().buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    get_fsm_context().actuator().SetEnable(true);
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
    auto& actuator = get_fsm_context().actuator();
    const uint32_t calib_elapsed = HAL_GetTick() - calib_start_time_;
    if (calib_elapsed > 2000) {
      return StateId::kFault;
    }
    actuator.gripper.SetSpeed(2.f * M_PI * 30.f);
    if (actuator.gripper.feedback().speed_rad_ps < 50.f &&
        actuator.gripper.feedback().power > 690 &&
        calib_elapsed > 250) {
      ++stall_count_;
    } else {
      stall_count_ = 0;
    }
    if (stall_count_ > 20) {
      actuator.gripper_encoder.Reset();
      actuator.gripper_encoder.Update(actuator.gripper.feedback().position_rad);
      actuator.calibrators[6].UpdateConfig({false, actuator.gripper_encoder.RealPosition()});
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
    get_fsm_context().actuator().SetEnable(false);
    get_fsm_context().shared().buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
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
    get_fsm_context().actuator().SetEnable(false, 1);
    return No_State_Change;
  }
  REACT(event::FaultClear) {
    UNUSED(e);
    get_fsm_context().arm_controller().FaultClear();
    return StateId::kNoForce;
  }
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state


}  // namespace fsm