#pragma once

#include "config.hpp"
#include "resources.hpp"
#include "fsm.hpp"

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

 private:
  // 这个数组需与fsm同生命周期，所以放在这里
  etl::ifsm_state *state_list_[9]  //
      {&state_no_force,
       &state_manual,
       &state_idle,
       &state_host_control,
       &state_custom_ctrl,
       &state_cartesian_custom_ctrl,
       &state_long_joint_calibrating,
       &state_sequencing,
       &state_fault};

 public:
  explicit Arm(const ArmConfig &config) : resources{config} {}

  /**
   * @brief 初始化（设备命名、回调注册）
   */
  void Init() {
    // // 设零点
    // HAL_Delay(1500);  // 达妙电机开机需要时间
    // resources.actuator.DmSetZero();

    auto &shared = SharedResources::GetInstance();
    // 1. 定义遥控器拨杆功能
    shared.rc(resources.config.rc_ch_mode_switch).OnValueChange([&](const int &old_value, const int &new_value) {
      // 353-up, 1024-mid, 1695-down
      if (old_value == 0) {
        return;  // 第一次触发不响应
      }
      if (new_value > 1300 && new_value < 1700) {  // 机械臂模式杆在下 - 无力
        fsm.receive(fsm_arm::event::FaultClear{});
        fsm.receive(fsm_arm::event::ForceModeSwitch{fsm_arm::StateId::kNoForce});
      }
      if (new_value > 0 && new_value < 500) {  // 机械臂模式杆在上 - 自动模式，idle
        fsm.receive(fsm_arm::event::ForceModeSwitch{fsm_arm::StateId::kIdle});
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
    fsm.set_states(state_list_, std::size(state_list_));
    fsm.start();
  }

  /**
   * @brief 500 Hz 控制循环节拍（IMU 数据从 SharedResources 内部读取）
   * @note  调用前须已调用 SharedResources::imu.Update()
   */
  void Tick() {
    resources.device_manager.Update();
    fsm.receive(fsm_arm::event::ControlLoop{resources.actuator.GetPositions(), resources.actuator.GetVelocities(), {}});
    rm::device::DjiMotorBase::SendCommand(*resources.config.can);
  }
};