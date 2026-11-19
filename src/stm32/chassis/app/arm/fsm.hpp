
#pragma once

#include <librm.hpp>
#include <etl/hfsm.h>

#include "resources.hpp"
#include "shared_resources.hpp"
#include "custom_controller.hpp"
#include "action_sequencer.hpp"
#include "harmonics_generator.hpp"
#include "sequences.hpp"

namespace fsm_arm {

struct EventId {
  enum {
    kControlLoop,        ///< 控制循环时钟事件
    kForceModeSwitch,    ///< 切换模式命令
    kCustomCtrlCommand,  ///< 自定义控制器命令
    kHostCommand,        ///< 上位机下发的命令
    kFaultClear,         ///< 故障清除命令
    kNextJoint,          ///< Manual模式下切换到下一个关节
    kActionSequence,     ///< 动作序列命令
  };
};

/// @brief 状态 ID，顺序必须与 Arm::state_list_ 的下标严格一致
struct StateId {
  enum : etl::fsm_state_id_t {
    // 顶层状态
    kNoForce,  ///< 强制无力
    kIdle,     ///< 空闲，可执行轨迹
    kMoving,   ///< 运动状态（HFSM 复合状态），内部包含 Manual/HostControl/CustomCtrl/Sequencing/Ident
    kFault,    ///< 故障状态

    // Moving 的子状态
    kManual,       ///< 遥控器手动模式，调试/检录的时候用
    kHostControl,  ///< 正在被上位机控制
    kCustomCtrl,   ///< 自定义控制器操作
    kSequencing,   ///< 正在执行动作序列
    kIdent,        ///< 用于参数辨识，执行固定激励轨迹

    NUMBER_OF_STATES,
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
struct SingleCustomCtrlCommand : etl::message<EventId::kCustomCtrlCommand>, ::SingleCustomCtrlCommand {
  SingleCustomCtrlCommand() = delete;
  explicit SingleCustomCtrlCommand(const ::SingleCustomCtrlCommand &command) : ::SingleCustomCtrlCommand(command) {}
};

/*******************************************/
struct SingleHostCommand : etl::message<EventId::kHostCommand> {
  SingleHostCommand(amorph::schema::ArmCommand cmd, bool is_left)
      : take_control(is_left ? cmd.take_control_left : cmd.take_control_right),
        gripper_open(is_left ? cmd.gripper_open_left : cmd.gripper_open_right) {
    for (int i = 0; i < 6; ++i) {
      q[i] = is_left ? cmd.left_joint_pos[i] : cmd.right_joint_pos[i];
    }
  }
  bool take_control{};
  bool gripper_open{};
  float q[6]{};
};

/*******************************************/
struct FaultClear : etl::message<EventId::kFaultClear> {};

/*******************************************/
struct NextJoint : etl::message<EventId::kNextJoint> {};

/*******************************************/
struct ActionSequence : etl::message<EventId::kActionSequence> {
  explicit ActionSequence(const ::ActionSequence &seq) : sequence(seq) {}
  ActionSequence(const ::ActionSequence &seq, etl::fsm_state_id_t finish_to_state)
      : sequence(seq), finish_to_state(finish_to_state) {}
  ::ActionSequence sequence;
  etl::fsm_state_id_t finish_to_state{StateId::kIdle};  ///< 执行完毕后转到哪个状态
};
}  // namespace event

/// @brief 机械臂分层状态机
/// @note  顶层为 NoForce/Idle/Moving/Fault；Moving 是复合状态，
///        其子状态在 Arm::Init() 中通过 set_child_states() 注册
class Arm : public etl::hfsm {
 public:
  explicit Arm(ArmResources &r) : hfsm(r.config.fsm_msg_router_id), resources_(&r) {}

  ArmResources &resources() const { return *resources_; }

  etl::fsm_state_id_t finish_to_state_{StateId::kIdle};  ///< 在Sequencing模式下执行完轨迹之后回到哪个状态

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
                                event::ControlLoop,               //
                                event::SingleHostCommand> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    resources.actuator.SetEnable(false);
    resources.actuator.ClearError();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    resources.controller.ResetAt(resources.actuator.GetPositions());
    resources.controller.Update(resources.actuator.GetPositions(), resources.actuator.GetVelocities(),
                                {shared.ahrs.imu_data().accel.x(), shared.ahrs.imu_data().accel.y(), shared.ahrs.imu_data().accel.z()},
                                0.002f);
    resources.actuator.ApplyOutput(ArmController::Output::Zero(), false);
    resources.actuator.SetEnable(false, 1);
    return No_State_Change;
  }
  REACT(event::SingleHostCommand) { return No_State_Change; }
};

/*******************************************/
/// @brief 运动复合状态（HFSM 父状态）
/// @note  子状态 Manual/HostControl/CustomCtrl/Sequencing/Ident 在 Arm::Init() 中通过
///        set_child_states() 注册，列表中首位 Manual 为默认子状态。
///        子状态未声明处理的事件会自动冒泡到此父状态处理。
struct Moving : etl::fsm_state<Arm, Moving, StateId::kMoving,  //
                               event::ForceModeSwitch> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &resources = get_fsm_context().resources();

    // 进入 Moving 时统一把规划器对齐到当前实测位置并使能电机。
    // 电机失能时编码器仍持续反馈，因此从 NoForce/Fault 恢复时这里读到的是有效位置。
    resources.controller.ResetAt(resources.actuator.GetPositions());
    resources.actuator.SetEnable(true);
    return No_State_Change;
  }
};

/*******************************************/
struct Manual : etl::fsm_state<Arm, Manual, StateId::kManual,  //
                               event::ControlLoop,             //
                               event::NextJoint,               //
                               event::ActionSequence> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    ctrl_joint_idx_ = 0;
    // 规划器重置与电机使能已由父状态 Moving 统一处理，
    // 此处直接以重置后的实测位置初始化目标，避免首帧位置跳变
    target_positions_ = resources.controller.current_positions();
    shared.buzzer_controller.Play<buzzer_melody::EnterManualMode>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    const auto &joint_limits = resources.controller.joint_limits();
    target_positions_[ctrl_joint_idx_] =
        std::clamp(target_positions_[ctrl_joint_idx_] + (shared.rc.right_y() * 0.001f),  //
                   joint_limits[ctrl_joint_idx_].first, joint_limits[ctrl_joint_idx_].second);
    // 控制夹爪开关
    resources.gripper_controller.Open(shared.rc.channel(resources.config.rc_ch_manual_gripper_switch) < 1000);
    resources.controller.GoTo(target_positions_);

    resources.controller.Update(resources.actuator.GetPositions(), resources.actuator.GetVelocities(),
                                {shared.ahrs.imu_data().accel.x(), shared.ahrs.imu_data().accel.y(), shared.ahrs.imu_data().accel.z()},
                                0.002f);
    resources.actuator.ApplyOutput(resources.controller.output(), true);
    resources.gripper_controller.Update(resources.actuator.GetGripperPosition(),
                                        resources.actuator.GetGripperVelocity(), resources.actuator.GetGripperPower());
    resources.actuator.gripper.SetMitCommand(0, 0, resources.gripper_controller.output(), 0, 0);
    // resources.actuator.gripper.SetMitCommand(0, 0, resources.gripper_controller.output(), 0, 0);
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
  REACT(event::ActionSequence) {
    auto &resources = get_fsm_context().resources();
    get_fsm_context().finish_to_state_ = e.finish_to_state;
    resources.sequencer.Execute(e.sequence);
    return StateId::kSequencing;
  }

 private:
  std::array<float, 6> target_positions_{};
  int ctrl_joint_idx_{0};
};

/*******************************************/
struct Idle : etl::fsm_state<Arm, Idle, StateId::kIdle,       //
                             event::ForceModeSwitch,          //
                             event::ControlLoop,              //
                             event::SingleHostCommand,        //
                             event::SingleCustomCtrlCommand,  //
                             event::ActionSequence> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    // 进入 Idle 时把规划器对齐到实测位置，防止上一次运动的目标残留导致位置跳变
    resources.controller.ResetAt(resources.actuator.GetPositions());
    shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    resources.actuator.SetEnable(true);
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();
    // if (resources.controller.status() == ArmController::kFault) {
    //   return StateId::kFault;
    // }

    resources.controller.Update(resources.actuator.GetPositions(), resources.actuator.GetVelocities(),
                                {shared.ahrs.imu_data().accel.x(), shared.ahrs.imu_data().accel.y(), shared.ahrs.imu_data().accel.z()},
                                0.002f);
    resources.actuator.ApplyOutput(resources.controller.output(), true);
    resources.gripper_controller.Update(resources.actuator.GetGripperPosition(),
                                        resources.actuator.GetGripperVelocity(), resources.actuator.GetGripperPower());
    resources.actuator.gripper.SetMitCommand(0, 0, resources.gripper_controller.output(), 0, 0);
    return No_State_Change;
  }
  REACT(event::SingleHostCommand) {
    if (e.take_control) {
      return StateId::kHostControl;
    }
    return No_State_Change;
  }
  REACT(event::SingleCustomCtrlCommand) {
    if (e.btn_a) {
      auto &resources = get_fsm_context().resources();
      get_fsm_context().finish_to_state_ = StateId::kIdle;
      resources.sequencer.Execute(arm_sequence::Reset(resources.actuator.GetPositions()));
      return StateId::kSequencing;
    }
    if (e.btn_b) {  // 按下按钮a，自定义控制器接管
      return StateId::kCustomCtrl;
    }
    return No_State_Change;
  }
  REACT(event::ActionSequence) {
    auto &resources = get_fsm_context().resources();
    get_fsm_context().finish_to_state_ = e.finish_to_state;
    resources.sequencer.Execute(e.sequence);
    return StateId::kSequencing;
  }
};

/*******************************************/
struct HostControl : etl::fsm_state<Arm, HostControl, StateId::kHostControl,  //
                                    event::ControlLoop,                       //
                                    event::SingleHostCommand> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();

    shared.buzzer_controller.Play<rm::modules::buzzer_melody::Success>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    switch (resources.controller.status()) {
      // case ArmController::kFault: {
      //   return StateId::kFault;
      // }
      case ArmController::kMoving:
      default: {
        // no state change
        break;
      }
    }
    resources.controller.Update(resources.actuator.GetPositions(), resources.actuator.GetVelocities(),
                                {shared.ahrs.imu_data().accel.x(), shared.ahrs.imu_data().accel.y(), shared.ahrs.imu_data().accel.z()},
                                0.002f);
    resources.actuator.ApplyOutput(resources.controller.output(), true);
    resources.gripper_controller.Update(resources.actuator.GetGripperPosition(),
                                        resources.actuator.GetGripperVelocity(), resources.actuator.GetGripperPower());
    resources.actuator.gripper.SetMitCommand(0, 0, resources.gripper_controller.output(), 0, 0);
    return No_State_Change;
  }
  REACT(event::SingleHostCommand) {
    auto &resources = get_fsm_context().resources();

    if (!e.take_control) {
      return StateId::kIdle;
    }
    std::array<float, 6> target_p = resources.controller.current_positions();
    for (int i = 0; i < 6; i++) {
      target_p[i] = e.q[i];
    }
    // 防止进入收纳区
    target_p[1] = std::min(e.q[1], (180_deg).rad() - std::atan2f(84.5f, 415.f) - (15_deg).rad());
    target_p[2] = std::max(e.q[2], -(180_deg).rad() + std::atan2f(84.5f, 415.f) + (20_deg).rad());
    resources.controller.GoTo(target_p);
    resources.gripper_controller.Open(e.gripper_open);
    return No_State_Change;
  }
};

/*******************************************/
struct CustomCtrl : etl::fsm_state<Arm, CustomCtrl, StateId::kCustomCtrl,  //
                                   event::ControlLoop,                     //
                                   event::SingleCustomCtrlCommand> {
  CustomCtrl() {
    btn_a_svw_.OnValueChange([this](const bool &old_value, const bool &new_value) {
      if (!old_value && new_value) {
        invert_j6_ = !invert_j6_;
      }
    });
  }
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();

    shared.buzzer_controller.Play<buzzer_melody::HiBeeps<1>>();
    invert_j6_ = false;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    // if (resources.controller.status() == ArmController::kFault) {
    //   return StateId::kFault;
    // }
    resources.controller.Update(resources.actuator.GetPositions(), resources.actuator.GetVelocities(),
                                {shared.ahrs.imu_data().accel.x(), shared.ahrs.imu_data().accel.y(), shared.ahrs.imu_data().accel.z()},
                                0.002f);
    resources.actuator.ApplyOutput(resources.controller.output(), true);
    resources.gripper_controller.Update(resources.actuator.GetGripperPosition(),
                                        resources.actuator.GetGripperVelocity(), resources.actuator.GetGripperPower());
    resources.actuator.gripper.SetMitCommand(0, 0, resources.gripper_controller.output(), 0, 0);
    return No_State_Change;
  }
  REACT(event::SingleCustomCtrlCommand) {
    auto &resources = get_fsm_context().resources();

    if (!e.btn_b) {  // 松开按钮b，退出CustomCtrl模式
      return StateId::kIdle;
    }

    btn_a_svw_.Update(e.btn_a);

    using namespace rm::modules::angle_literals;
    // p[1] = (180_deg).rad() - std::atan2f(84.5f, 415.f) - (15_deg).rad();
    // p[2] = -(180_deg).rad() + std::atan2f(84.5f, 415.f) + (20_deg).rad();
    std::array<float, 6> target_p;
    target_p[0] = e.q[0];
    target_p[1] = std::min(e.q[1], (180_deg).rad() - std::atan2f(84.5f, 415.f) - (35_deg).rad());
    target_p[2] = std::max(e.q[2], -(180_deg).rad() + std::atan2f(84.5f, 415.f) + (20_deg).rad());
    target_p[3] = e.q[3];
    target_p[4] = e.q[4];
    if (invert_j6_) {
      target_p[5] = e.q[5] + M_PIf;
    } else {
      target_p[5] = e.q[5];
    }
    resources.controller.GoTo(target_p);
    resources.gripper_controller.Open(!e.btn_c);
    return No_State_Change;
  }

 private:
  rm::modules::SparseValueWatcher<bool, true> btn_a_svw_;
  bool invert_j6_;
};

/*******************************************/
struct Sequencing : etl::fsm_state<Arm, Sequencing, StateId::kSequencing,  //
                                   event::ForceModeSwitch,                 //
                                   event::ControlLoop,                     //
                                   event::FaultClear,                      //
                                   event::ActionSequence> {
  // 仅允许切到 NoForce；其余模式切换事件由父状态 Moving 统一处理后忽略
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();

    shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    return No_State_Change;
  }
  REACT(event::ForceModeSwitch) {
    if (e.target_mode == StateId::kNoForce) {
      return StateId::kNoForce;
    }
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();

    // Tick sequencer，推进动作序列
    constexpr float kDt = 0.002f;
    constexpr uint32_t kDtMs = 2;
    resources.sequencer.Update(kDtMs);

    // 更新 ArmController 并输出
    resources.controller.Update(resources.actuator.GetPositions(), resources.actuator.GetVelocities(),
                                {shared.ahrs.imu_data().accel.x(), shared.ahrs.imu_data().accel.y(), shared.ahrs.imu_data().accel.z()}, kDt);
    resources.actuator.ApplyOutput(resources.controller.output(), true);

    // 更新夹爪
    resources.gripper_controller.Update(resources.actuator.GetGripperPosition(),
                                        resources.actuator.GetGripperVelocity(), resources.actuator.GetGripperPower());
    resources.actuator.gripper.SetMitCommand(0, 0, resources.gripper_controller.output(), 0, 0);

    // 序列结束，回到指定状态
    if (resources.sequencer.IsFinished()) {
      return get_fsm_context().finish_to_state_;
    }
    return No_State_Change;
  }
  REACT(event::FaultClear) {
    UNUSED(e);
    auto &resources = get_fsm_context().resources();

    resources.controller.FaultClear();
    return StateId::kNoForce;
  }
  REACT(event::ActionSequence) {
    // 动作序列可以被打断
    auto &resources = get_fsm_context().resources();
    get_fsm_context().finish_to_state_ = e.finish_to_state;
    resources.sequencer.Execute(e.sequence);
    return No_State_Change;
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

/*******************************************/
struct Ident : etl::fsm_state<Arm, Ident, StateId::kIdent,  //
                              event::ControlLoop> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();

    shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    auto &resources = get_fsm_context().resources();
    // if (resources.controller.status() == ArmController::kFault) {
    //   return StateId::kFault;
    // }

    for (int i = 0; i < 3; ++i) {
      target_[i] = tg_[i].evaluate(HAL_GetTick() / 1000.f).q;
    }
    auto current_pos = resources.actuator.GetPositions();
    current_pos[0] = target_[0];
    current_pos[1] = target_[1];
    current_pos[2] = target_[2];

    resources.controller.GoTo(current_pos);
    resources.controller.Update(resources.actuator.GetPositions(), resources.actuator.GetVelocities(),
                                {shared.ahrs.imu_data().accel.x(), shared.ahrs.imu_data().accel.y(), shared.ahrs.imu_data().accel.z()},
                                0.002f);
    resources.actuator.ApplyOutput(resources.controller.output(), true);
    resources.gripper_controller.Update(resources.actuator.GetGripperPosition(),
                                        resources.actuator.GetGripperVelocity(), resources.actuator.GetGripperPower());
    resources.actuator.gripper.SetMitCommand(0, 0, resources.gripper_controller.output(), 0, 0);
    return No_State_Change;
  }

 private:
  std::array<float, 6> target_{};
  FourierTrajectoryGenerator<5> tg_[3]{
      {0.25f,
       -1.570796 / 2.f,
       0.523599,
       {0.025f, 0.035f, 0.030f, 0.020f, 0.015f},
       {0.030f, 0.025f, 0.020f, 0.025f, 0.015f}},
      {0.167f, 1.7, 2.35, {0.08f, 0.10f, 0.08f, 0.06f, 0.04f}, {0.06f, 0.08f, 0.10f, 0.06f, 0.04f}},
      {0.1f, -2.26, -0.6, {0.10f, 0.08f, 0.06f, 0.08f, 0.05f}, {0.06f, 0.10f, 0.08f, 0.05f, 0.05f}},
  };
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm_arm