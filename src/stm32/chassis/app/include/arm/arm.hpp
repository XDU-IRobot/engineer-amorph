#pragma once

#include "config.hpp"
#include "resources.hpp"
#include "fsm.hpp"
#include "sequences.hpp"

struct Arm {
  ArmResources resources;

  // 状态机
  fsm_arm::Arm fsm{resources};
  fsm_arm::state::NoForce state_no_force;
  fsm_arm::state::Manual state_manual;
  fsm_arm::state::Idle state_idle;
  fsm_arm::state::HostControl state_host_control;
  fsm_arm::state::CustomCtrl state_custom_ctrl;
  fsm_arm::state::Sequencing state_sequencing;
  fsm_arm::state::Fault state_fault;
  fsm_arm::state::Ident state_ident;

 private:
  // 这个数组需与fsm同生命周期，所以放在这里
  etl::ifsm_state *state_list_[8]  //
      {
          &state_no_force,      //
          &state_manual,        //
          &state_idle,          //
          &state_host_control,  //
          &state_custom_ctrl,   //
          &state_sequencing,    //
          &state_fault,         //
          &state_ident,
      };

 public:
  explicit Arm(const ArmConfig &config) : resources{config} {}

  /**
   * @brief 初始化（设备命名、回调注册）
   */
  void Init() {
    auto &shared = SharedResources::GetInstance();

    // if (resources.config.arm_id == ArmConfig::kLeft) {
    //   HAL_Delay(3000);
    //   resources.actuator.DmSetZero();
    // }

    // 定义遥控器拨杆功能
    shared.rc(resources.config.rc_ch_mode_switch).OnValueChange([&](const int &old_value, const int &new_value) {
      // 353-up, 1024-mid, 1695-down
      if (old_value == 0) {
        return;  // 第一次触发不响应
      }
      if (new_value > 1300 && new_value < 1700) {  // 机械臂模式杆在下 - 无力
        fsm.receive(fsm_arm::event::FaultClear{});
        fsm.receive(fsm_arm::event::ForceModeSwitch{fsm_arm::StateId::kNoForce});
      }
      if (new_value > 0 && new_value < 500) {  // 机械臂模式杆在上 - idle
        fsm.receive(fsm_arm::event::ForceModeSwitch{fsm_arm::StateId::kIdle});
        fsm.receive(fsm_arm::event::ActionSequence{arm_sequence::Reset(resources.actuator.GetPositions())});
      }
      if (new_value > 500 && new_value < 1100) {  // 机械臂模式杆在中 - 手动调试模式
        fsm.receive(fsm_arm::event::ForceModeSwitch{fsm_arm::StateId::kManual});
      }
    });
    shared.rc(resources.config.rc_ch_next_joint).OnValueChange([&](const int &old_value, const int &new_value) {
      UNUSED(old_value);
      if (new_value > 1000) {  // SH按下
        fsm.receive(fsm_arm::event::NextJoint{});
      }
    });

    // 夹爪校准完成回调：更新编码器零点和校准器配置
    resources.gripper_controller.OnCalibrationComplete([&]() {
      // resources.actuator.gripper_encoder.Reset();
      // resources.actuator.gripper_encoder.Update(resources.actuator.gripper.feedback().position_rad);
      resources.actuator.calibrators[6].UpdateConfig({false, resources.actuator.gripper.feedback().position_multi_rad});
    });

    resources.device_manager.OnDeviceStatusChange([&](rm::device::Device *dev) {
      if (dev->name() == "gripper" && dev->online_status() != rm::device::Device::kOk) {
        resources.gripper_controller.SetNeedsCalibration(true);
      }
    });

    // 定义键盘键位
    using RC = rm::device::VT03::KeyboardKey;
    const auto register_sequence_hotkey = [&](uint16_t modifier) {
      shared.kh.OnKeyDown(RC::kX, modifier, [&](const rm::modules::KeyEvent &e) {
        UNUSED(e);
        if (fsm.get_state_id() != fsm_arm::StateId::kIdle) {  // 非手动模式禁用
          return;
        }
        shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
        fsm.receive(fsm_arm::event::ActionSequence{arm_sequence::LeftTake2(resources.actuator.GetPositions())});
      });
      shared.kh.OnKeyDown(RC::kC, modifier, [&](const rm::modules::KeyEvent &e) {
        UNUSED(e);
        if (fsm.get_state_id() != fsm_arm::StateId::kIdle) {  // 非手动模式禁用
          return;
        }
        shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
        fsm.receive(fsm_arm::event::ActionSequence{arm_sequence::RightTake3(resources.actuator.GetPositions())});
      });
    };
    if (resources.config.arm_id == ArmConfig::kLeft) {
      shared.kh.OnKeyDown(RC::kQ, [&](const rm::modules::KeyEvent &e) {
        UNUSED(e);
        resources.gripper_controller.SetNeedsCalibration(true);
      });
      register_sequence_hotkey(RC::kCtrl);
    } else if (resources.config.arm_id == ArmConfig::kRight) {
      shared.kh.OnKeyDown(RC::kE, [&](const rm::modules::KeyEvent &e) {
        UNUSED(e);
        resources.gripper_controller.SetNeedsCalibration(true);
      });
      register_sequence_hotkey(RC::kShift);
    }
    fsm.set_states(state_list_, std::size(state_list_));
    fsm.start();
  }

  /**
   * @brief 500 Hz 控制循环节拍（IMU 数据从 SharedResources 内部读取）
   * @note  调用前须已调用 SharedResources::imu.Update()
   */
  void Tick() {
    resources.device_manager.Update();
    resources.actuator.QueryEncoders();
    fsm.receive(fsm_arm::event::ControlLoop{});
    rm::device::DjiMotorBase::SendCommand(*resources.config.can);
  }
};