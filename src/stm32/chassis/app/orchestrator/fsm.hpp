
#pragma once

#include <etl/fsm.h>

#include "shared_resources.hpp"
#include "global_objects.hpp"

namespace fsm_orch {

struct EventId {
  enum {
    kControlLoop,        ///< 控制循环时钟事件
    kCustomCtrlCommand,  ///< 自定义控制器命令
    kSwitchConfigReq,    ///< 切换构型请求
    kHostCommand,        ///< 机械臂上位机命令
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kDisabled,   ///< 非比赛模式，Orchestrator不启用
    kEnabling,   ///< 启动Orchestrator，正在复位到正常构型
    kNormal,     ///< 正常构型
    kRetracted,  ///< 收纳构型
    kFault,      ///< 故障
  };
};

namespace event {
/*******************************************/
struct ControlLoop : etl::message<EventId::kControlLoop> {};

/*******************************************/
struct CustomCtrlCommand : etl::message<EventId::kCustomCtrlCommand>, ::CustomControllerCommand {
  CustomCtrlCommand() = default;
  explicit CustomCtrlCommand(const CustomControllerCommand &command) : CustomControllerCommand(command) {}
};

/*******************************************/
struct SwitchConfigReq : etl::message<EventId::kSwitchConfigReq> {
  explicit SwitchConfigReq(etl::fsm_state_id_t target_config) : target_config(target_config) {}
  etl::fsm_state_id_t target_config;
};

/*******************************************/
struct HostCommand : etl::message<EventId::kHostCommand>, amorph::schema::ArmCommand {
  HostCommand() : ArmCommand{} {}
  explicit HostCommand(const ArmCommand &cmd) : ArmCommand{cmd} {}
};

}  // namespace event

class OrchestratorFsm : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = GlobalConfig::kOrchMessageRouterId;

 public:
  OrchestratorFsm() : fsm(kMessageRouterId) {}

  /// @brief 是否满足强制急停条件？
  static bool CheckIfEmergency() {
    auto &shared = SharedResources::GetInstance();

    // 遥控器、VT03图传均离线
    if (shared.rc.online_status() != rm::device::Device::kOk &&  //
        shared.vt_rc.online_status() != rm::device::Device::kOk) {
      return true;
    }

    const bool left_arm_ok = left_arm->resources.device_manager.all_device_ok();
    const bool right_arm_ok = right_arm->resources.device_manager.all_device_ok();
    const bool chassis_ok = chassis->resources.device_manager.all_device_ok();
    const bool gimbal_ok = gimbal->resources.device_manager.all_device_ok();
    // 有子系统设备离线
    if (!left_arm_ok ||   //
        !right_arm_ok ||  //
        !chassis_ok ||    //
        !gimbal_ok) {
      return true;
    }

    return false;
  }

  /// @brief 是否满足进入比赛模式的条件？
  static bool CheckIfOkToStart() {
    auto &shared = SharedResources::GetInstance();

    if (CheckIfEmergency()) {
      return false;
    }

    // 遥控器四个杆不全在上
    if (WflyET16s::SwitchPos(shared.rc.channel(rc_ch::SA)) != WflyET16s::kUp ||
        WflyET16s::SwitchPos(shared.rc.channel(rc_ch::SB)) != WflyET16s::kUp ||
        WflyET16s::SwitchPos(shared.rc.channel(rc_ch::SC)) != WflyET16s::kUp ||
        WflyET16s::SwitchPos(shared.rc.channel(rc_ch::SD)) != WflyET16s::kUp) {
      return false;
    }

    // 所有检查通过
    return true;
  }
  /// @brief 当已经处于比赛模式下，Orchestrator在运行时，是否还满足运行条件？
  static bool CheckIfOkToContinue() {
    auto &shared = SharedResources::GetInstance();

    if (!CheckIfOkToStart()) {
      return false;
    }

    return true;
  }

  static bool IsUserForcingDisable() {
    auto &shared = SharedResources::GetInstance();
    // 遥控器四个杆不全在上
    if (WflyET16s::SwitchPos(shared.rc.channel(rc_ch::SA)) != WflyET16s::kUp ||
        WflyET16s::SwitchPos(shared.rc.channel(rc_ch::SB)) != WflyET16s::kUp ||
        WflyET16s::SwitchPos(shared.rc.channel(rc_ch::SC)) != WflyET16s::kUp ||
        WflyET16s::SwitchPos(shared.rc.channel(rc_ch::SD)) != WflyET16s::kUp) {
      return true;
    }
    return false;
  }

  /// @param vel_rotate_rad 把 [vx vy] 向量旋转多少
  /// @param spin_vel       强制给一个旋转速度，用于小陀螺时
  fsm_chassis::event::VelocityCommand GenerateChassisCmd(float vel_rotate_rad = 0.f, float spin_vel = 0.f) {
    auto &shared = SharedResources::GetInstance();

    static bool any_custom_ctrl_enable = false;
    bool custom_ctrl_enable_this_time =
        shared.custom_controller.online_status() == rm::device::Device::kOk &&
        (shared.custom_controller.data().left_btn_b || shared.custom_controller.data().right_btn_b);
    if (!any_custom_ctrl_enable && custom_ctrl_enable_this_time) {
      initial_rpy_[0] = shared.client_broker.joy_data().imu_roll;
      initial_rpy_[1] = shared.client_broker.joy_data().imu_pitch;
      initial_rpy_[2] = shared.client_broker.joy_data().imu_yaw;
    }
    any_custom_ctrl_enable =
        shared.custom_controller.online_status() == rm::device::Device::kOk &&
        (shared.custom_controller.data().left_btn_b || shared.custom_controller.data().right_btn_b);

    using RC = rm::device::VT03::KeyboardKey;
    const uint16_t keycode = shared.vt_rc.data().keyboard_key;
    bool move_gently_custom_ctrl{false};
    move_gently_custom_ctrl = any_custom_ctrl_enable;
    const bool move_gently_kbd = ((keycode & RC::kShift) != 0);  // 按住Shift键，慢速移动
    const float key_w = ((keycode & RC::kW) ? 1.f : 0.f);
    const float key_s = ((keycode & RC::kS) ? 1.f : 0.f);
    const float key_a = ((keycode & RC::kA) ? 1.f : 0.f);
    const float key_d = ((keycode & RC::kD) ? 1.f : 0.f);
    const float key_q = ((keycode & RC::kC) ? 1.f : 0.f);
    const float key_e = ((keycode & RC::kB) ? 1.f : 0.f);

    // 遥控器
    const float rc_x = shared.rc.left_x();
    const float rc_y = shared.rc.left_y();
    const float rc_w = shared.rc.right_x();

    // 选手端键盘
    const float kbd_x = key_d - key_a;
    const float kbd_y = key_w - key_s;
    const float kbd_w = 0.f;

    // 选手端鼠标
    const float vt_w = shared.vt_rc.data().mouse_x / 660.f;

    float client_x{0.f}, client_y{0.f}, client_w{0.f};
    float imu_x{0.f}, imu_y{0.f}, imu_w{0.f};
    const float xbox_move_multiplier = shared.xbox_controller.GetButton(xbox_btn::BTN_A) ? 1.f : 0.5f;
    bool move_gently_xbox_controller{false};
    if (shared.client_broker.online_status() == rm::device::Device::kOk) {
      // 自定义客户端 xbox手柄
      const auto &client_data = shared.xbox_controller.data();
      using rm::modules::Deadzone;
      using std::clamp;
      client_x = clamp(Deadzone(client_data.left_x, -0.05f, 0.05f) * xbox_move_multiplier, -1.0f, 1.0f);
      client_y = clamp(Deadzone(client_data.left_y, -0.05f, 0.05f) * xbox_move_multiplier, -1.0f, 1.0f);
      client_w = clamp(Deadzone(client_data.right_x, -0.05f, 0.05f) * xbox_move_multiplier, -1.0f, 1.0f);

      move_gently_xbox_controller = shared.xbox_controller.GetButton(xbox_btn::BTN_LB);

      if (any_custom_ctrl_enable) {
        // 自定义控制器使能后，启用imu头追
        float command_rpy[3];
        command_rpy[0] = rm::modules::AngleDelta(client_data.imu_roll, initial_rpy_[0]);
        command_rpy[1] = rm::modules::AngleDelta(client_data.imu_pitch, initial_rpy_[1]);
        command_rpy[2] = rm::modules::AngleDelta(rm::modules::Wrap(client_data.imu_yaw, -M_PIf, M_PIf),
                                                 rm::modules::Wrap(initial_rpy_[2], -M_PIf, M_PIf));
        constexpr float kRpyThreshold[3]{(15_deg).rad(), (15_deg).rad(), (30_deg).rad()};
        imu_x = std::abs(command_rpy[0]) > kRpyThreshold[0] ? std::clamp(command_rpy[0] * 3.f, -1.f, 1.f) : 0.f;
        imu_y = std::abs(command_rpy[1]) > kRpyThreshold[1] ? std::clamp(command_rpy[1] * 3.f, -1.f, 1.f) : 0.f;
        imu_w = std::abs(command_rpy[2]) > kRpyThreshold[2] ? std::clamp(command_rpy[2] * 3.f, -1.f, 1.f) : 0.f;
      }
    }

    const float target_vx = (rc_x + kbd_x + client_x - imu_x) * 1000.f;
    const float target_vy = (rc_y + kbd_y + client_y - imu_y) * 1000.f;
    const float target_w = -(rc_w + kbd_w + client_w + vt_w + imu_w) * 5.f;

    // 按 vel_rotate_rad 旋转速度向量 (target_vx, target_vy)
    const float cos_rot = std::cos(vel_rotate_rad);
    const float sin_rot = std::sin(vel_rotate_rad);
    const float rotated_vx = target_vx * cos_rot - target_vy * sin_rot;
    const float rotated_vy = target_vx * sin_rot + target_vy * cos_rot;

    using rm::modules::IsNear;

    fsm_chassis::event::VelocityCommand ret;
    ret.move_gently = move_gently_custom_ctrl || move_gently_kbd || move_gently_xbox_controller;
    const float slow_mod = ret.move_gently ? 0.07f : 1.f;
    const float slow_mod_w = ret.move_gently ? 0.1f : 1.f;
    ret.vx = rotated_vx * slow_mod;
    ret.vy = rotated_vy * slow_mod;
    if (IsNear(spin_vel, 0.f, 1e-4)) {
      ret.w = target_w * slow_mod_w;
    } else {
      ret.w = spin_vel;
    }
    return ret;
  }

  etl::fsm_state_id_t Transit(etl::fsm_state_id_t target_state) {
    prev_state_ = get_state_id();
    return target_state;
  }

  float initial_rpy_[3];
  etl::fsm_state_id_t prev_state_{};
};

namespace state {

///*********************************
///*******  helper macros  *********
///*********************************
#define IGNORE_UNINTEREST_EVENT() \
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &) { return No_State_Change; }
#define ENTER etl::fsm_state_id_t on_enter_state()
#define REACT(EventType) etl::fsm_state_id_t on_event(const EventType &e)
#define TRANSIT(target_state) return get_fsm_context().Transit(target_state)
///*********************************
///*********************************
///*********************************

/*******************************************/
struct Disabled : etl::fsm_state<OrchestratorFsm, Disabled, StateId::kDisabled,  //
                                 event::ControlLoop> {
  IGNORE_UNINTEREST_EVENT();
  ENTER { return No_State_Change; }
  REACT(event::ControlLoop) {
    if (OrchestratorFsm::CheckIfEmergency()) {
      TRANSIT(StateId::kFault);
    }
    if (OrchestratorFsm::CheckIfOkToStart()) {
      TRANSIT(StateId::kEnabling);
    }
    chassis->fsm.receive(get_fsm_context().GenerateChassisCmd());
    return No_State_Change;
  }
};

/*******************************************/
struct Enabling : etl::fsm_state<OrchestratorFsm, Enabling, StateId::kEnabling,  //
                                 event::ControlLoop> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    gimbal->fsm.receive(fsm_gimbal::event::ModeSwitch{fsm_gimbal::StateId::kUp});
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    if (OrchestratorFsm::CheckIfEmergency()) {
      TRANSIT(StateId::kFault);
    }
    if (OrchestratorFsm::IsUserForcingDisable()) {
      TRANSIT(StateId::kDisabled);
    }
    if (OrchestratorFsm::CheckIfOkToContinue()) {
      TRANSIT(StateId::kNormal);
    }
    return No_State_Change;
  }
};

/*******************************************/
struct Normal : etl::fsm_state<OrchestratorFsm, Normal, StateId::kNormal,  //
                               event::ControlLoop,                         //
                               event::CustomCtrlCommand,                   //
                               event::SwitchConfigReq,                     //
                               event::HostCommand> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    auto &shared = SharedResources::GetInstance();

    if (get_fsm_context().prev_state_ == StateId::kRetracted) {
      pending_arm_action_ = true;
    }
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    auto &shared = SharedResources::GetInstance();
    if (OrchestratorFsm::CheckIfEmergency()) {
      TRANSIT(StateId::kFault);
    }
    if (!OrchestratorFsm::CheckIfOkToContinue()) {
      TRANSIT(StateId::kDisabled);
    }

    // if (shared.xbox_controller.online_status() == rm::device::Device::kOk) {
    //   const auto &xbox_data = shared.xbox_controller.data();
    //   using rm::modules::Deadzone;
    //   using rm::modules::IsNear;
    //   const float triggers = Deadzone(xbox_data.lt, -0.05f, 0.05f) - Deadzone(xbox_data.rt, -0.05f, 0.05f);
    //   if (!IsNear(triggers, 0.f, 0.2f)) {
    //     TRANSIT(StateId::kSpin);
    //   }
    // }

    // 存矿取矿
    using RC = rm::device::VT03::KeyboardKey;
    static std::atomic<bool> ongoing_pick_or_place{false};
    if (ongoing_pick_or_place.load()) {
      gimbal->fsm.receive(fsm_gimbal::event::PitchCommand{false});
    } else {
      gimbal->fsm.receive(fsm_gimbal::event::PitchCommand{true});
    }
    if (left_arm->resources.sequencer.IsFinished() && right_arm->resources.sequencer.IsFinished()) {
      ongoing_pick_or_place.store(false);
    }
    const uint16_t ctrl_x = RC::kX | RC::kCtrl;
    const uint16_t ctrl_c = RC::kC | RC::kCtrl;
    if (((shared.vt_rc.data().keyboard_key & ctrl_x) == ctrl_x) &&  // CTRL+X
        (!shared.eu_storage.slot_2())) {                            // 左边没有存矿
      const bool ret = left_arm->TryExec(arm_sequence::LeftStore(left_arm->resources.actuator.GetPositions()));
      if (ret) {
        ongoing_pick_or_place.store(true);
      }
    }
    if ((shared.vt_rc.data().keyboard_key & ctrl_c) == ctrl_c) {
      const bool ret = left_arm->TryExec(arm_sequence::LeftTake(left_arm->resources.actuator.GetPositions()));
      if (ret) {
        ongoing_pick_or_place.store(true);
      }
    }
    const uint16_t shift_x = RC::kX | RC::kShift;
    const uint16_t shift_c = RC::kC | RC::kShift;
    if (((shared.vt_rc.data().keyboard_key & shift_x) == shift_x) &&  // SHIFT+X
        (!shared.eu_storage.slot_1())) {                              // 右边没有存矿
      const bool ret = right_arm->TryExec(arm_sequence::RightStore(right_arm->resources.actuator.GetPositions()));
      if (ret) {
        ongoing_pick_or_place.store(true);
      }
    }
    if ((shared.vt_rc.data().keyboard_key & shift_c) == shift_c) {
      const bool ret = right_arm->TryExec(arm_sequence::RightTake(right_arm->resources.actuator.GetPositions()));
      if (ret) {
        ongoing_pick_or_place.store(true);
      }
    }

    if (shared.client_broker.online_status() == rm::device::Device::kOk) {
      if (shared.xbox_controller.GetButton(xbox_btn::BTN_LS)) {  // 收左臂
        left_arm->fsm.receive(
            fsm_arm::event::ActionSequence{arm_sequence::Reset(left_arm->resources.actuator.GetPositions())});
      }
      if (shared.xbox_controller.GetButton(xbox_btn::BTN_RS)) {  // 收右臂
        right_arm->fsm.receive(
            fsm_arm::event::ActionSequence{arm_sequence::Reset(right_arm->resources.actuator.GetPositions())});
      }
      if (shared.xbox_controller.GetButton(xbox_btn::BTN_VIEW)) {  // 左臂抬到垂直
        left_arm->fsm.receive(
            fsm_arm::event::ActionSequence{arm_sequence::FuckOff(right_arm->resources.actuator.GetPositions())});
      }
      if (shared.xbox_controller.GetButton(xbox_btn::BTN_MENU)) {  // 右臂抬到垂直
        right_arm->fsm.receive(
            fsm_arm::event::ActionSequence{arm_sequence::FuckOff(right_arm->resources.actuator.GetPositions())});
      }
      if (shared.xbox_controller.data().btn_x) {
        TRANSIT(StateId::kRetracted);
      }
    }

    if (pending_arm_action_ &&  //
        gimbal->resources.lift_tl.IsAtTarget()) {
      pending_arm_action_ = false;
      left_arm->fsm.receive(
          fsm_arm::event::ActionSequence{arm_sequence::Reset(left_arm->resources.actuator.GetPositions())});
      right_arm->fsm.receive(
          fsm_arm::event::ActionSequence{arm_sequence::Reset(right_arm->resources.actuator.GetPositions())});
    }

    if (shared.client_broker.online_status() == rm::device::Device::kOk &&
        shared.xbox_controller.GetButton(xbox_btn::BTN_RB)) {
      chassis->fsm.receive(get_fsm_context().GenerateChassisCmd(M_PIf));
    } else {
      chassis->fsm.receive(get_fsm_context().GenerateChassisCmd(0.f));
    }
    return No_State_Change;
  }
  REACT(event::CustomCtrlCommand) {
    auto &shared = SharedResources::GetInstance();
    if (!e.ok) {
      return No_State_Change;
    }
    left_arm->fsm.receive(fsm_arm::event::SingleCustomCtrlCommand{SingleCustomCtrlCommand{e, true}});
    right_arm->fsm.receive(fsm_arm::event::SingleCustomCtrlCommand{SingleCustomCtrlCommand{e, false}});
    return No_State_Change;
  }

  REACT(event::SwitchConfigReq) {
    if (e.target_config == StateId::kRetracted) {
      left_arm->fsm.receive(
          fsm_arm::event::ActionSequence{arm_sequence::Retract(left_arm->resources.actuator.GetPositions(), true)});
      right_arm->fsm.receive(
          fsm_arm::event::ActionSequence{arm_sequence::Retract(right_arm->resources.actuator.GetPositions(), false)});
      TRANSIT(StateId::kRetracted);
    }
    return No_State_Change;
  }

  REACT(event::HostCommand) {
    left_arm->fsm.receive(fsm_arm::event::SingleHostCommand{static_cast<amorph::schema::ArmCommand>(e), true});
    right_arm->fsm.receive(fsm_arm::event::SingleHostCommand{static_cast<amorph::schema::ArmCommand>(e), false});
    return No_State_Change;
  }

 private:
  bool pending_arm_action_{false};
};

/*******************************************/
struct Retracted : etl::fsm_state<OrchestratorFsm, Retracted, StateId::kRetracted,  //
                                  event::ControlLoop,                               //
                                  event::SwitchConfigReq> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    lift_action_pending_ = true;
    left_arm->fsm.receive(
        fsm_arm::event::ActionSequence{arm_sequence::Retract(left_arm->resources.actuator.GetPositions(), true)});
    right_arm->fsm.receive(
        fsm_arm::event::ActionSequence{arm_sequence::Retract(right_arm->resources.actuator.GetPositions(), false)});
    SharedResources::GetInstance().buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    auto &shared = SharedResources::GetInstance();

    if (OrchestratorFsm::CheckIfEmergency()) {
      TRANSIT(StateId::kFault);
    }
    if (!OrchestratorFsm::CheckIfOkToContinue()) {
      TRANSIT(StateId::kDisabled);
    }

    if (shared.xbox_controller.data().btn_y) {
      gimbal->fsm.receive(fsm_gimbal::event::ModeSwitch{fsm_gimbal::StateId::kUp});
      TRANSIT(StateId::kNormal);
    }

    if (lift_action_pending_ &&  //
        left_arm->fsm.get_state_id() == fsm_arm::StateId::kIdle &&
        right_arm->fsm.get_state_id() == fsm_arm::StateId::kIdle) {
      lift_action_pending_ = false;
      gimbal->fsm.receive(fsm_gimbal::event::ModeSwitch{fsm_gimbal::StateId::kDown});
    }
    chassis->fsm.receive(get_fsm_context().GenerateChassisCmd());
    return No_State_Change;
  }

  REACT(event::SwitchConfigReq) {
    if (e.target_config == StateId::kNormal) {
      gimbal->fsm.receive(fsm_gimbal::event::ModeSwitch{fsm_gimbal::StateId::kUp});
      TRANSIT(StateId::kNormal);
    }
    return No_State_Change;
  }

 private:
  bool lift_action_pending_{false};
};

/*******************************************/
struct Fault : etl::fsm_state<OrchestratorFsm, Fault, StateId::kFault,  //
                              event::ControlLoop> {
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    SharedResources::GetInstance().buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    UNUSED(e);
    if (!OrchestratorFsm::CheckIfEmergency()) {
      TRANSIT(StateId::kDisabled);
    }
    return No_State_Change;
  }
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm_orch