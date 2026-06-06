#pragma once

#include <librm.hpp>

#include "config.hpp"
#include "arm_controller.hpp"
#include "gripper_controller.hpp"
#include "action_sequencer.hpp"
#include "actuators_legacy.hpp"

struct ArmResources {
  explicit ArmResources(const ArmConfig &config)
      : config{config},
        actuator{config},
        controller{config.joint_limits, config.trajectory_limiters, config.dynamics_params} {
    actuator.j1.SetName("j1");
    actuator.j2.SetName("j2");
    actuator.j3.SetName("j3");
    actuator.j4.SetName("j4");
    actuator.j5.SetName("j5");
    actuator.j6.SetName("j6");
    actuator.gripper.SetName("gripper");

    device_manager << &actuator.j1 << &actuator.j2 << &actuator.j3 << &actuator.j4 << &actuator.j5 << &actuator.j6
                   << &actuator.gripper;
  }

  ArmConfig config;

  ActuatorLegacy actuator;
  ArmController controller;                                   ///< 轨迹规划 + 控制输出计算（6轴关节）
  GripperController<> gripper_controller;                     ///< 夹爪控制器（独立于6轴控制器，内置校准）
  ActionSequencer sequencer{controller, gripper_controller};  ///< 动作序列执行器

  rm::device::DeviceManager<10> device_manager;  ///< 管理所有电机的在线状态
};