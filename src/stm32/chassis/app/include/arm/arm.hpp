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
  fsm_arm::state::CartesianCustomCtrl state_cartesian_custom_ctrl;
  fsm_arm::state::LongJointCalibrating state_long_joint_calibrating;
  fsm_arm::state::Sequencing state_sequencing;
  fsm_arm::state::Fault state_fault;
  fsm_arm::state::Ident state_ident;

 private:
  // 这个数组需与fsm同生命周期，所以放在这里
  etl::ifsm_state *state_list_[10]  //
      {
          &state_no_force,
          &state_manual,
          &state_idle,
          &state_host_control,
          &state_custom_ctrl,
          &state_cartesian_custom_ctrl,
          &state_long_joint_calibrating,
          &state_sequencing,
          &state_fault,
          &state_ident,
      };

 public:
  explicit Arm(const ArmConfig &config) : resources{config} {}

  /**
   * @brief 初始化（设备命名、回调注册）
   */
  void Init() {
    // if (resources.config.arm_id == ArmConfig::kRight) {
    //   // 设零点
    //   resources.actuator.DmSetZero();
    // }

    auto &shared = SharedResources::GetInstance();

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
        // fsm.receive(fsm_arm::event::ActionSequence{arm_sequence::Reset(resources.actuator.GetPositions())});
      }
    });
    shared.rc(resources.config.rc_ch_next_joint).OnValueChange([&](const int &old_value, const int &new_value) {
      UNUSED(old_value);
      if (new_value > 1000) {  // SH按下
        fsm.receive(fsm_arm::event::NextJoint{});
      }
    });

    // 初始化图传链路串口
    // 注意，此处涉及到的串口和DMA的中断优先级应该和TIM13保持一致，使两者互斥，防止同时操作状态机
    shared.ref_vt_link.AttachCallback([&](uint16_t cmd_id, uint8_t seq) {
      UNUSED(seq);
      const auto &data = shared.ref_vt_link.data().custom_robot_data.data;
      switch (cmd_id) {
        case rm::device::RefereeCmdId<GlobalConfig::kRefereeRev>::kCustomRobotData: {
          resources.custom_ctrl_device.ReportStatus(rm::device::Device::kOk);
          const bool is_cartesian_cmd = data[0] & 0xff;
          if (is_cartesian_cmd) {
            CartesianCustomControllerCommand cmd;
            std::memcpy(&cmd, data, sizeof(cmd));
            // fsm.receive(fsm_arm::event::CartesianCustomCtrlCommand{cmd});
          } else {
            static CustomControllerCommand cc_cmd;
            std::memcpy(&cc_cmd, data, sizeof(cc_cmd));
            fsm.receive(fsm_arm::event::CustomCtrlCommand{cc_cmd});
            int a = std::clamp(1, 1, 1);
          }
          break;
        }
        default: {
          break;
        }
      }
    });

    resources.device_manager.OnDeviceStatusChange([&](rm::device::Device *dev) {
      if (dev->name() == "gripper" && dev->online_status() != rm::device::Device::kOk) {
        resources.gripper_calibrated = false;
        fsm.receive(fsm_arm::event::ForceModeSwitch{fsm_arm::StateId::kNoForce});
      }
    });

    // 定义键盘键位
    using RC = rm::device::RefereeProtocol<GlobalConfig::kRefereeRev>::RemoteControl;
    shared.kh.OnKeyDown(RC::kV, RC::kCtrl, [&](const rm::modules::KeyEvent &e) {
      UNUSED(e);
      shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
      fsm.receive(fsm_arm::event::ActionSequence{arm_sequence::Retract(resources.actuator.GetPositions())});
    });
    shared.kh.OnKeyDown(RC::kF, RC::kCtrl, [&](const rm::modules::KeyEvent &e) {
      UNUSED(e);
      shared.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
      fsm.receive(fsm_arm::event::ActionSequence{arm_sequence::Reset(resources.actuator.GetPositions())});
    });
    fsm.set_states(state_list_, std::size(state_list_));
    fsm.start();
  }

  /**
   * @brief 500 Hz 控制循环节拍（IMU 数据从 SharedResources 内部读取）
   * @note  调用前须已调用 SharedResources::imu.Update()
   */
  void Tick() {
    resources.device_manager.Update();
    resources.actuator.QueryGripperEncoder();
    fsm.receive(fsm_arm::event::ControlLoop{});
    rm::device::DjiMotorBase::SendCommand(*resources.config.can);
  }
};