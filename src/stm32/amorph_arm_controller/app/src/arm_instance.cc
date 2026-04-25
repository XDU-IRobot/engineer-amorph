/**
 * @file  arm_instance.cc
 * @brief ArmInstance 构造函数及初始化方法实现
 */

#include "arm_instance.hpp"

// ============================================================
//  构造函数
// ============================================================
ArmInstance::ArmInstance(const ArmConfig& config, SharedResources& shared)
    : actuator{*config.lower_motor_can, *config.upper_motor_can, config},
      arm_controller{config.joint_limits,
                     MakeTrajectoryLimiters(config.trajectory_params),
                     config.dynamics_b1,
                     config.dynamics_b2},
      host{shared.comm_can, config.host_comm_id},
      rc_ch_mode_switch{config.rc_ch_mode_switch},
      rc_ch_next_joint{config.rc_ch_next_joint} {
  // 设置 FSM 上下文
  arm_fsm.SetArmData({&arm_controller, &actuator, &shared});

  // 注册状态实例（使用成员数组确保生命期安全）
  state_list_[0] = &state_no_force;
  state_list_[1] = &state_manual;
  state_list_[2] = &state_idle;
  state_list_[3] = &state_host_control;
  state_list_[4] = &state_custom_ctrl;
  state_list_[5] = &state_cartesian_custom_ctrl;
  state_list_[6] = &state_long_joint_calibrating;
  state_list_[7] = &state_fault;
  arm_fsm.set_states(state_list_, 8);
  arm_fsm.start();
}

// ============================================================
//  初始化
// ============================================================
void ArmInstance::Init(SharedResources& shared) {
  // ---- 设备命名（方便离线日志定位） ----
  actuator.j1.SetName("motor_j1");
  actuator.j2.SetName("motor_j2");
  actuator.j3.SetName("motor_j3");
  actuator.j4.SetName("motor_j4");
  actuator.j5.SetName("motor_j5");
  actuator.j6.SetName("motor_j6");
  actuator.gripper.SetName("motor_gripper");

  // ---- 注册设备状态变化事件回调 ----
  device_manager.OnDeviceStatusChange(
      [this](rm::device::Device* /*dev*/) { status_str = device_manager.GetSummaryString(); });

  host_connection_watcher.OnDeviceStatusChange([this, &shared](rm::device::Device* /*dev*/) {
    if (host_connection_watcher.all_device_ok()) {
      shared.buzzer_controller.Play<rm::modules::buzzer_melody::Success>();
    } else {
      shared.buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
    }
  });

  // ---- 把设备注册给设备管理器 ----
  device_manager << &actuator.j1 << &actuator.j2 << &actuator.j3
                 << &actuator.j4 << &actuator.j5 << &actuator.j6
                 << &actuator.gripper;
  host_connection_watcher << &host;

  // 注意：电机 CAN 总线（SetFilter + Begin）需要在 main.cc 中对具体的
  //       ThrottledCan 对象调用，因为 CanInterface 基类没有这两个方法。
}

// ============================================================
//  控制循环
// ============================================================
void ArmInstance::Tick() {
  device_manager.Update();
  host_connection_watcher.Update();
  actuator.UpdateEncoderCounter();
  arm_fsm.receive(fsm::event::ControlLoop{});
}

void ArmInstance::TickSlow() {
  host.tx_data_.mode =
      static_cast<Stm32FeedbackCompressed::Mode>(arm_fsm.get_state_id());
  const auto pos = actuator.GetPositions();
  const auto vel = actuator.GetVelocities();
  const auto eff = actuator.GetEfforts();
  std::memcpy(host.tx_data_.joint_positions,  pos.data(), sizeof(float) * 6);
  std::memcpy(host.tx_data_.joint_velocities, vel.data(), sizeof(float) * 6);
  std::memcpy(host.tx_data_.joint_efforts,    eff.data(), sizeof(float) * 6);
  host.tx_data_.gripper_state =
      static_cast<Stm32FeedbackCompressed::GripperState>(arm_controller.gripper().state());
  host.Write();
}




