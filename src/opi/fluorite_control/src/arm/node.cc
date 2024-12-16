/**
 * @file  fluorite_chassis.cc
 * @brief 控制节点
 */

#include "arm/node.hpp"

#include <chrono>
#include <cmath>
#include <memory>

#include <fo_lpf.h>
#include <librm.hpp>
#include <tinyfsm.hpp>

#include "arm/fsm.hpp"
#include "common/logger.hpp"

using namespace std::chrono_literals;
using namespace rm;
using namespace rm::modules::algorithm::utils;

FluoriteArm::FluoriteArm(const rclcpp::NodeOptions &options) : rclcpp::Node("fluorite_control_arm", options) {
  ros_params_manager_.Init();  // 读取yaml文件中的参数、启动参数服务器
  this->InitFsm();             // 初始化状态机
  this->DeviceMonitor();       // 设备在线状态监控
  this->Remote();              // 启动遥控器接收
  this->Referee();             // 启动裁判系统接收
  this->Navigation();          // 启动和导航节点的通信
  this->Blackbox();            // 启动黑匣子（广播state和control）

  // 启动控制循环
  info("control node initialize ok");
  control_loop_timer_ = this->create_wall_timer(2.5ms, [this] {
    this->UpdateFsmState();
    this->ParamHotUpdate();
    fsm::DispatchEvent(control_msg_);
    device::DjiMotor<>::SendCommand();
  });
  info("CONTROL UP");
}

FluoriteArm::~FluoriteArm() { fsm::DispatchEvent(event::arm::SwitchMode<state::arm::NoForce>{}); }

void FluoriteArm::InitFsm() {
  auto arm_can = new hal::Can(ros_params_manager_.data().arm_can_port.c_str());

  arm_can->SetFilter(0, 0);  // 设置can过滤器
  arm_can->Begin();          // 开启can总线
  fsm::fsm_list::start();    // 状态机开始监听事件
  info("fsm init ok");
}

void FluoriteArm::Navigation() { info("navigation ok"); }

void FluoriteArm::Remote() { info("remote ok"); }

void FluoriteArm::Referee() { info("referee ok"); }

/**
 * @brief 字面意思
 */
void FluoriteArm::Blackbox() {
  blackbox_state_pub_ =
      this->create_publisher<fluorite_msgs::msg::ArmState>("blackbox/arm_state", rclcpp::SystemDefaultsQoS());
  blackbox_timer_ = this->create_wall_timer(10ms, [this] { blackbox_state_pub_->publish(fsm::Arm::state); });
  info("blackbox ok");
}

/**
 * @brief 设备在线状态监控
 * @note  因为irobotec还没有推进到这里所以暂时弄了一种很蹩脚的方法，只监控遥控器和imu
 */
void FluoriteArm::DeviceMonitor() {
  device_monitor_ << remote_device_;
  device_monitor_timer_ = this->create_wall_timer(300ms, [this] {
    device_monitor_.Update();
    if (!device_monitor_.all_device_ok()) {
      fsm::DispatchEvent(event::arm::SwitchMode<state::arm::NoForce>{});
    }
  });
  info("device monitor ok");
}

/**
 * @brief 把电机对象里的保存的电机状态复制到控制事件里
 */
void FluoriteArm::UpdateFsmState() {}

void FluoriteArm::ParamHotUpdate() {}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteArm>());
  rclcpp::shutdown();
  return 0;
}