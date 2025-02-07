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
#if __has_include("wiringPi.h")
# include <wiringPi.h>
#endif

#include "arm/fsm.hpp"
#include "common/logger.hpp"

using namespace std::chrono_literals;
using namespace rm;
using namespace rm::modules::algorithm::utils;

FluoriteArm::FluoriteArm(const rclcpp::NodeOptions &options) : rclcpp::Node("fluorite_control_arm", options) {
  ros_params_manager_.Init();  // 读取yaml文件中的参数、启动参数服务器
#if __has_include("wiringPi.h")
  wiringPiSetup();             // 初始化wiringPi
#endif
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
    // device::DjiMotor<>::SendCommand();
  });
  info("CONTROL UP");
}

FluoriteArm::~FluoriteArm() { fsm::DispatchEvent(event::arm::SwitchMode<state::arm::NoForce>{}); }

void FluoriteArm::InitFsm() {
  auto arm_can = new hal::Can(ros_params_manager_.data().arm_can_port.c_str());
  fsm::Arm::actuator.base_yaw =
      std::make_unique<rm::device::DirectDriveMotor>(*arm_can, ros_params_manager_.data().base_yaw_id);
  fsm::Arm::actuator.yaw.reset(
      new device::DmMotor<device::DmMotorControlMode::kMit>(*arm_can,
                                                            {.master_id = ros_params_manager_.data().yaw_master_id,
                                                             .slave_id = ros_params_manager_.data().yaw_slave_id,
                                                             .p_max = 12.5f,
                                                             .v_max = 45.f,
                                                             .t_max = 54.f,
                                                             .kp_range = {0, 500},
                                                             .kd_range = {0, 50}},
                                                            ros_params_manager_.data().yaw_reverse));
  fsm::Arm::actuator.pitch_1.reset(
      new device::DmMotor<device::DmMotorControlMode::kMit>(*arm_can,
                                                            {.master_id = ros_params_manager_.data().pitch_1_master_id,
                                                             .slave_id = ros_params_manager_.data().pitch_1_slave_id,
                                                             .p_max = 12.5f,
                                                             .v_max = 45.f,
                                                             .t_max = 54.f,
                                                             .kp_range = {0, 500},
                                                             .kd_range = {0, 50}},
                                                            ros_params_manager_.data().pitch_1_reverse));
  fsm::Arm::actuator.pitch_2.reset(
      new device::DmMotor<device::DmMotorControlMode::kMit>(*arm_can,
                                                            {.master_id = ros_params_manager_.data().pitch_2_master_id,
                                                             .slave_id = ros_params_manager_.data().pitch_2_slave_id,
                                                             .p_max = 12.5f,
                                                             .v_max = 45.f,
                                                             .t_max = 54.f,
                                                             .kp_range = {0, 500},
                                                             .kd_range = {0, 50}},
                                                            ros_params_manager_.data().pitch_2_reverse));
  fsm::Arm::actuator.end_yaw_1.reset(new device::DmMotor<device::DmMotorControlMode::kMit>(
      *arm_can,
      {.master_id = ros_params_manager_.data().end_yaw_1_master_id,
       .slave_id = ros_params_manager_.data().end_yaw_1_slave_id,
       .p_max = 12.5f,
       .v_max = 30.f,
       .t_max = 10.f,
       .kp_range = {0, 500},
       .kd_range = {0, 50}},
      ros_params_manager_.data().end_yaw_1_reverse));
  fsm::Arm::actuator.end_pitch.reset(
      new device::DmMotor<device::DmMotorControlMode::kMit>(*arm_can, {.master_id = 0xc7,
                                                                       .slave_id = 0xc8,
                                                                       .p_max = 12.5f,
                                                                       .v_max = 30.f,
                                                                       .t_max = 10.f,
                                                                       .kp_range = {0, 500},
                                                                       .kd_range = {0, 50}}));
  fsm::Arm::actuator.end_yaw_2 = std::make_unique<rm::device::GM6020>(*arm_can, ros_params_manager_.data().end_yaw_2_id,
                                                                      ros_params_manager_.data().end_yaw_2_reverse);
  fsm::Arm::actuator.sucker_system =
      std::make_unique<SuckerSystem>(Pin<OUTPUT>(ros_params_manager_.data().sucker_out_valve_pin),
                                     Pin<OUTPUT>(ros_params_manager_.data().sucker_in_valve_pin),
                                     Pin<PWM_OUTPUT>(ros_params_manager_.data().sucker_vaccum_pump_pin));
  fsm::Arm::actuator.storage_system =
      std::make_unique<StorageSystem>(Pin<OUTPUT>(ros_params_manager_.data().storage_left_valve_pin),
                                      Pin<OUTPUT>(ros_params_manager_.data().storage_right_valve_pin),
                                      Pin<PWM_OUTPUT>(ros_params_manager_.data().storage_vaccum_pump_pin));

  arm_can->SetFilter(0, 0);  // 设置can过滤器
  arm_can->Begin();          // 开启can总线
  fsm::fsm_list::start();    // 状态机开始监听事件
  info("fsm init ok");
}

void FluoriteArm::Navigation() { info("navigation ok"); }

void FluoriteArm::Remote() {
  static std::array<short, 16> last_channels{};
  remote_sub_ = this->create_subscription<sbus_interface::msg::Sbus>(
      "/sbus", rclcpp::SensorDataQoS(), [&](const sbus_interface::msg::Sbus::SharedPtr msg) {
        // remote_device_.IAmAlive();
        if (msg->failsafe) {
          fsm::DispatchEvent(event::arm::SwitchMode<state::arm::NoForce>{});
          return;
        }
        if (msg->mapped_channels[11] == -1000) {
          if ((last_channels[11] != -1000)) {  // SA up
            fsm::Arm::actuator.sucker_system->Suck(0.7);
            info("sucker: suck 0.7");
          }
        }
        if (msg->mapped_channels[11] == 0) {
          if ((last_channels[11] != 0)) {  // SA mid
            fsm::Arm::actuator.sucker_system->Disable();
            info("sucker: disable");
          }
        }
        if (msg->mapped_channels[11] == 1000) {
          if ((last_channels[11] != 1000)) {  // SA down
            fsm::Arm::actuator.sucker_system->Blow(0.7);
            info("sucker: blow 0.7");
          }
        }
        last_channels = msg->mapped_channels;
      });
  info("remote ok");
}

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
 */
void FluoriteArm::DeviceMonitor() {
  device_monitor_timer_ = this->create_wall_timer(300ms, [this] {
    // device_monitor_.Update();
    // if (!device_monitor_.all_device_ok()) {
    //   fsm::DispatchEvent(event::arm::SwitchMode<state::arm::NoForce>{});
    // }
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