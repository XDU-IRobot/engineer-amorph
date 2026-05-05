
#pragma once

#include <librm.hpp>
#include <etl/fsm.h>

#include "host_comm.hpp"
#include "kinematics.hpp"
#include "resources.hpp"
#include "shared_resources.hpp"
#include "custom_controller_protocol.hpp"

namespace fsm_arm {

struct EventId {
  enum {
    kControlLoop,                 ///< 控制循环时钟事件
    kForceModeSwitch,             ///< 切换模式命令
    kCustomCtrlCommand,           ///< 自定义控制器命令
    kCartesianCustomCtrlCommand,  ///< 笛卡尔空间自定义控制器命令（T265）
    kHostCommand,                 ///< 上位机下发的命令
    kFaultClear,                  ///< 故障清除命令
    kNextJoint,                   ///< Manual模式下切换到下一个关节
    kJointPositionSequence,       ///< 关节位置序列命令
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    // kInit, ///<
    kNoForce,               ///< 强制无力
    kManual,                ///< 遥控器手动模式，调试/检录的时候用
    kIdle,                  ///< 空闲，可执行轨迹
    kHostControl,           ///< 正在被上位机控制
    kCustomCtrl,            ///< 自定义控制器操作
    kCartesianCustomCtrl,   ///< 笛卡尔空间自定义控制器操作（T265）
    kLongJointCalibrating,  ///< 夹爪校准位置
    kSequencing,            ///< 正在执行动作序列
    kFault,                 ///< 故障状态
  };
};

namespace event {
/*******************************************/
struct ControlLoop : etl::message<EventId::kControlLoop> {
  ControlLoop(const std::array<float, 7> &positions,   //
              const std::array<float, 7> &velocities,  //
              Eigen::Vector3f imu_acc)
      : current_positions(positions), current_velocities(velocities), imu_acc(imu_acc) {}
  std::array<float, 7> current_positions;
  std::array<float, 7> current_velocities;
  Eigen::Vector3f imu_acc;  ///< 来自IMU的加速度数据，单位：m/s^2，机械臂坐标系（前左上）下的分量
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

/*******************************************/
struct JointPositionSequence : etl::message<EventId::kJointPositionSequence> {
  constexpr static size_t kMaxPoints = 5;  ///< 最多5个点，为了省空间尽量小
  using Points = std::array<std::array<float, 6>, kMaxPoints>;

  explicit JointPositionSequence(const Points &points) : points{points} {}
  Points points;
};
}  // namespace event

class Arm : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = 0;

 public:
  explicit Arm(ArmResources &r) : fsm(kMessageRouterId), resources_(&r) {}

  bool planner_need_reset_{
      true};  ///< 从故障中恢复或刚上电的时候，规划器不知道当前关节位置/丢失跟踪了一段时间，需要在下一个ControlLoop到来的时候强制把规划器的当前位置重置为当前关节位置

  ArmResources &resources() const { return *resources_; }

 private:
  ArmResources *resources_;
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
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    get_fsm_context().planner_need_reset_ = true;
    shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    resources.actuator.SetEnable(false);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();

    resources.controller.ResetAt(e.current_positions);
    resources.controller.Update(e.current_positions,                          //
                                e.current_velocities,                         //
                                resources.actuator.gripper.feedback().power,  //
                                e.imu_acc,                                    //
                                0.002f);
    resources.actuator.gripper.SendInstruction(
        rm::device::LkMotor<rm::device::LkMotorType::kMS, 15>::kReadPositionMulti);
    resources.actuator.SetEnable(false, 1);
    return No_State_Change;
  }
};

/*******************************************/
struct Manual : etl::fsm_state<Arm, Manual, StateId::kManual,  //
                               event::ForceModeSwitch,         //
                               event::ControlLoop,             //
                               event::NextJoint> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    ctrl_joint_idx_ = 0;
    // 进入Manual时强制重置规划器，防止位置跳变
    get_fsm_context().planner_need_reset_ = true;
    // 注意: target_positions_ 的初始化移动到第一个 ControlLoop 中进行，
    // 因为此时 planner_need_reset_ 为 true，controller.output() 不是最新位置
    target_positions_initialized_ = false;
    shared.buzzer_controller.Play<buzzer_melody::EnterManualMode>();
    resources.actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    if (get_fsm_context().planner_need_reset_) {
      resources.controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    // 确保在 ResetAt 之后再初始化 target_positions_，避免位置跳变
    if (!target_positions_initialized_) {
      target_positions_ = resources.controller.current_positions();
      target_positions_initialized_ = true;
    }
    const auto &joint_limits = resources.controller.joint_limits();
    target_positions_[ctrl_joint_idx_] =
        std::clamp(target_positions_[ctrl_joint_idx_] + (shared.rc.right_y() * 0.001f),  //
                   joint_limits[ctrl_joint_idx_].first, joint_limits[ctrl_joint_idx_].second);
    target_positions_[6] = (shared.rc.channel(rc_ch::SF) < 1000) ? 1.f : 0.f;  // sf控制夹爪开关
    resources.controller.GoTo(target_positions_);

    resources.controller.Update(e.current_positions, e.current_velocities, resources.actuator.gripper.feedback().power,
                                e.imu_acc, 0.002f);

    auto output = resources.controller.output();
    resources.actuator.ApplyOutput(output, true);
    return No_State_Change;
  }
  REACT(event::NextJoint) {
    UNUSED(e);
    auto &shared = SharedResources::GetInstance();
    ctrl_joint_idx_ = (ctrl_joint_idx_ + 1) % 6;
    switch (ctrl_joint_idx_) {
        // clang-format off
      case 0: shared.buzzer_controller.Play<buzzer_melody::HiBeeps<1>>(); break;
      case 1: shared.buzzer_controller.Play<buzzer_melody::HiBeeps<2>>(); break;
      case 2: shared.buzzer_controller.Play<buzzer_melody::HiBeeps<3>>(); break;
      case 3: shared.buzzer_controller.Play<buzzer_melody::HiBeeps<4>>(); break;
      case 4: shared.buzzer_controller.Play<buzzer_melody::HiBeeps<5>>(); break;
      case 5: shared.buzzer_controller.Play<buzzer_melody::HiBeeps<6>>(); break;
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
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    get_fsm_context().planner_need_reset_ = true;  // 进入Idle时强制重置规划器，防止位置跳变
    shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    resources.actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();
    // if (resources.controller.status() == ArmController<7>::kFault) {
    //   return StateId::kFault;
    // }
    if (get_fsm_context().planner_need_reset_) {
      resources.controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }

    resources.controller.Update(e.current_positions, e.current_velocities, resources.actuator.gripper.feedback().power,
                                e.imu_acc, 0.002f);
    resources.actuator.ApplyOutput(resources.controller.output(), true);
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
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    if (e.btn_a) {  // 按下按钮a，自定义控制器接管
      return StateId::kCartesianCustomCtrl;
    }
    if (e.btn_b) {  // 没有按A使能自定义控制器的情况下，按B使机械臂归位
      shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
      resources.controller.GoTo({0.f, 2.8f, -2.74f, 0.f, 0.f, 0.f, 1.f});
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
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    shared.buzzer_controller.Play<rm::modules::buzzer_melody::Success>();
    resources.actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();

    switch (resources.controller.status()) {
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
      resources.controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    resources.controller.Update(e.current_positions, e.current_velocities, resources.actuator.gripper.feedback().power,
                                e.imu_acc, 0.002f);
    resources.actuator.ApplyOutput(resources.controller.output(), true);
    return No_State_Change;
  }
  REACT(event::HostCommand) {
    auto &resources = get_fsm_context().resources();

    if (!e.take_control) {
      return StateId::kIdle;
    }
    std::array<float, 7> target_p = resources.controller.current_positions();
    target_p[0] = e.target_joint_positions[0];
    target_p[1] = e.target_joint_positions[1];
    target_p[2] = e.target_joint_positions[2];
    target_p[3] = e.target_joint_positions[3];
    target_p[4] = e.target_joint_positions[4];
    target_p[5] = e.target_joint_positions[5];
    target_p[6] = e.gripper_open ? 1.f : 0.f;
    resources.controller.GoTo(target_p);
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
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    get_fsm_context().planner_need_reset_ = true;  // 强制重置规划器，防止位置跳变
    shared.buzzer_controller.Play<buzzer_melody::HiBeeps<1>>();
    resources.actuator.SetEnable(true);
    just_entered_ = true;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();

    // if (resources.controller.status() == ArmController<7>::kFault) {
    //   return StateId::kFault;
    // }
    if (get_fsm_context().planner_need_reset_) {
      resources.controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    resources.controller.Update(e.current_positions, e.current_velocities, resources.actuator.gripper.feedback().power,
                                e.imu_acc, 0.002f);
    resources.actuator.ApplyOutput(resources.controller.output(), true);
    return No_State_Change;
  }
  REACT(event::CustomCtrlCommand) {
    auto &resources = get_fsm_context().resources();

    if (!e.buttons.a) {  // 松开按钮a，退出CustomCtrl模式
      return StateId::kIdle;
    }
    if (!e.ok) {
      return No_State_Change;
    }
    if (just_entered_) {
      j6_delta_ = e.joint_positions[5] - resources.controller.current_positions()[5];
      just_entered_ = false;
    }
    std::array<float, 7> target_p = resources.controller.current_positions();
    target_p[0] = e.joint_positions[0];
    target_p[1] = e.joint_positions[1];
    target_p[2] = e.joint_positions[2];
    target_p[3] = e.joint_positions[3];
    target_p[4] = e.joint_positions[4];
    target_p[5] = e.joint_positions[5] - j6_delta_;
    target_p[6] = e.buttons.b ? 0.f : 1.f;
    resources.controller.GoTo(target_p);
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
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    get_fsm_context().planner_need_reset_ = true;  // 强制重置规划器，防止位置跳变
    shared.buzzer_controller.Play<buzzer_melody::HiBeeps<1>>();
    resources.actuator.SetEnable(true);
    // 进入笛卡尔空间自定义控制模式时，记录当前末端位置作为初始位置，后续的控制命令都是相对于这个初始位置的增量
    Eigen::Matrix3f dummy;
    const auto &current_p = resources.actuator.GetPositions();
    k_.SolveFk({current_p[0], current_p[1], current_p[2], current_p[3], current_p[4], current_p[5]},
               end_eff_initial_pos_, dummy);
    just_entered_ = true;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &resources = get_fsm_context().resources();

    // if (resources.controller.status() == ArmController<7>::kFault) {
    //   return StateId::kFault;
    // }
    if (get_fsm_context().planner_need_reset_) {
      resources.controller.ResetAt(e.current_positions);
      get_fsm_context().planner_need_reset_ = false;
    }
    resources.controller.Update(e.current_positions, e.current_velocities, resources.actuator.gripper.feedback().power,
                                e.imu_acc, 0.002f);
    resources.actuator.ApplyOutput(resources.controller.output(), true);
    return No_State_Change;
  }
  REACT(event::CartesianCustomCtrlCommand) {
    auto &resources = get_fsm_context().resources();

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
        resources.controller.current_positions()[0], resources.controller.current_positions()[1],
        resources.controller.current_positions()[2], resources.controller.current_positions()[3],
        resources.controller.current_positions()[4], resources.controller.current_positions()[5]};
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
    resources.controller.GoTo(target_p_);
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
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    stall_count_ = 0;
    calib_start_time_ = HAL_GetTick();
    shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    resources.actuator.SetEnable(true);
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
    auto &resources = get_fsm_context().resources();

    const uint32_t calib_elapsed = HAL_GetTick() - calib_start_time_;
    if (calib_elapsed > 2000) {
      // 长时间没有检测到堵转，认为校准失败，进入故障状态
      return StateId::kFault;
    }
    resources.actuator.gripper.SetSpeed(2.f * M_PI * 30.f);
    if (resources.actuator.gripper.feedback().speed_rad_ps < 50.f &&  // 转速低
        resources.actuator.gripper.feedback().power > 690 &&          // 力矩大
        calib_elapsed > 250                                           // 过了初始加速的阶段
    ) {
      // 认为电机堵转，增加一次堵转计数
      ++stall_count_;
    } else {
      stall_count_ = 0;
    }
    if (stall_count_ > 20) {  // 连续多次堵转，认为校准完成，更新编码器零点，并进入手动模式
      resources.actuator.gripper_encoder.Reset();
      resources.actuator.gripper_encoder.Update(resources.actuator.gripper.feedback().position_rad);
      resources.actuator.calibrators[6].UpdateConfig({false, resources.actuator.gripper_encoder.RealPosition()});
      return StateId::kManual;
    }
    return No_State_Change;
  }
  size_t stall_count_{0};
  uint32_t calib_start_time_{0};
};

/*******************************************/
struct Sequencing : etl::fsm_state<Arm, Sequencing, StateId::kSequencing,  //
                                   event::ForceModeSwitch,                 //
                                   event::ControlLoop,                     //
                                   event::FaultClear> {
  // ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
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
    auto &resources = get_fsm_context().resources();

    resources.actuator.SetEnable(false, 1);
    return No_State_Change;
  }
  REACT(event::FaultClear) {
    UNUSED(e);
    auto &resources = get_fsm_context().resources();

    resources.controller.FaultClear();
    return StateId::kNoForce;
  }
};

/*******************************************/
struct Fault : etl::fsm_state<Arm, Fault, StateId::kFault,  //
                              event::ForceModeSwitch,       //
                              event::ControlLoop,           //
                              event::FaultClear> {
  // ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    get_fsm_context().planner_need_reset_ = true;
    resources.actuator.SetEnable(false);
    shared.buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
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
    auto &resources = get_fsm_context().resources();

    resources.actuator.SetEnable(false, 1);
    return No_State_Change;
  }
  REACT(event::FaultClear) {
    UNUSED(e);
    auto &resources = get_fsm_context().resources();

    resources.controller.FaultClear();
    return StateId::kNoForce;
  }
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm_arm