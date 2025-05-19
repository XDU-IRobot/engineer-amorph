/**
 * @file  chassis/node.cc
 * @brief 底盘控制节点
 */

#include "chassis/node.hpp"

#include <chrono>
#include <cmath>
#include <memory>

#include <fo_lpf.h>
#include <librm.hpp>
#include <tinyfsm.hpp>

#include "chassis/fsm.hpp"
#include "common/logger.hpp"

namespace fluorite::chassis {

using namespace std::chrono_literals;
using namespace rm;
using namespace rm::modules::algorithm::utils;

FluoriteChassis::FluoriteChassis(rclcpp::NodeOptions const &options)
    : rclcpp::Node("fluorite_control_chassis", options) {
  ros_params_manager_.Init();  // 读取yaml文件中的参数、启动参数服务器
  this->InitFsm();             // 初始化状态机
  this->DeviceMonitor();       // 设备在线状态监控
  this->UserInput();           // 启动遥控器接收
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

FluoriteChassis::~FluoriteChassis() { fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::NoForce>{}); }

void FluoriteChassis::InitFsm() {
  auto chassis_can = new hal::Can(ros_params_manager_.data().chassis_can_port.c_str());
  auto arm_can = new hal::Can(ros_params_manager_.data().arm_can_port.c_str());

  // clang-format off
  fsm::Chassis::actuator.base_yaw = std::make_unique<device::DirectDriveMotor>(*arm_can, 1);
  fsm::Chassis::actuator.lf_wheel = std::make_unique<device::M3508>(
      *chassis_can, 
      ros_params_manager_.data().lf_wheel_id, 
      ros_params_manager_.data().lf_wheel_reverse);
  fsm::Chassis::actuator.rf_wheel = std::make_unique<device::M3508>(
      *chassis_can, 
      ros_params_manager_.data().rf_wheel_id, 
      ros_params_manager_.data().rf_wheel_reverse);
  fsm::Chassis::actuator.lr_wheel = std::make_unique<device::M3508>(
      *chassis_can, 
      ros_params_manager_.data().lr_wheel_id, 
      ros_params_manager_.data().lr_wheel_reverse);
  fsm::Chassis::actuator.rr_wheel = std::make_unique<device::M3508>(
      *chassis_can, 
      ros_params_manager_.data().rr_wheel_id, 
      ros_params_manager_.data().rr_wheel_reverse);
  fsm::Chassis::actuator.lf_steer = std::make_unique<device::GM6020>(
      *chassis_can, 
      ros_params_manager_.data().lf_steer_id, 
      ros_params_manager_.data().lf_steer_reverse);
  fsm::Chassis::actuator.rf_steer = std::make_unique<device::GM6020>(
      *chassis_can, 
      ros_params_manager_.data().rf_steer_id, 
      ros_params_manager_.data().rf_steer_reverse);
  fsm::Chassis::actuator.lr_steer = std::make_unique<device::GM6020>(
      *chassis_can, 
      ros_params_manager_.data().lr_steer_id, 
      ros_params_manager_.data().lr_steer_reverse);
  fsm::Chassis::actuator.rr_steer = std::make_unique<device::GM6020>(
      *chassis_can, 
      ros_params_manager_.data().rr_steer_id, 
      ros_params_manager_.data().rr_steer_reverse);

  using PosPid = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;
  using PosRingPid = rm::modules::algorithm::RingPID<rm::modules::algorithm::PIDType::kPosition>;
 
  fsm::Chassis::controller.lf_wheel_speed = std::make_unique<PosPid>(
      ros_params_manager_.data().pid_lf_wheel_speed[0], 
      ros_params_manager_.data().pid_lf_wheel_speed[1],
      ros_params_manager_.data().pid_lf_wheel_speed[2],
      ros_params_manager_.data().pid_lf_wheel_speed[3],
      ros_params_manager_.data().pid_lf_wheel_speed[4]);
  fsm::Chassis::controller.rf_wheel_speed = std::make_unique<PosPid>(
      ros_params_manager_.data().pid_rf_wheel_speed[0], 
      ros_params_manager_.data().pid_rf_wheel_speed[1],
      ros_params_manager_.data().pid_rf_wheel_speed[2],
      ros_params_manager_.data().pid_rf_wheel_speed[3],
      ros_params_manager_.data().pid_rf_wheel_speed[4]);
  fsm::Chassis::controller.lr_wheel_speed = std::make_unique<PosPid>(
      ros_params_manager_.data().pid_lr_wheel_speed[0], 
      ros_params_manager_.data().pid_lr_wheel_speed[1],
      ros_params_manager_.data().pid_lr_wheel_speed[2],
      ros_params_manager_.data().pid_lr_wheel_speed[3],
      ros_params_manager_.data().pid_lr_wheel_speed[4]);
  fsm::Chassis::controller.rr_wheel_speed = std::make_unique<PosPid>(
      ros_params_manager_.data().pid_rr_wheel_speed[0], 
      ros_params_manager_.data().pid_rr_wheel_speed[1],
      ros_params_manager_.data().pid_rr_wheel_speed[2],
      ros_params_manager_.data().pid_rr_wheel_speed[3],
      ros_params_manager_.data().pid_rr_wheel_speed[4]);
  fsm::Chassis::controller.lf_steer_speed = std::make_unique<PosPid>(
      ros_params_manager_.data().pid_lf_steer_speed[0], 
      ros_params_manager_.data().pid_lf_steer_speed[1],
      ros_params_manager_.data().pid_lf_steer_speed[2],
      ros_params_manager_.data().pid_lf_steer_speed[3],
      ros_params_manager_.data().pid_lf_steer_speed[4]);
  fsm::Chassis::controller.rf_steer_speed = std::make_unique<PosPid>(
      ros_params_manager_.data().pid_rf_steer_speed[0], 
      ros_params_manager_.data().pid_rf_steer_speed[1],
      ros_params_manager_.data().pid_rf_steer_speed[2],
      ros_params_manager_.data().pid_rf_steer_speed[3],
      ros_params_manager_.data().pid_rf_steer_speed[4]);
  fsm::Chassis::controller.lr_steer_speed = std::make_unique<PosPid>(
      ros_params_manager_.data().pid_lr_steer_speed[0], 
      ros_params_manager_.data().pid_lr_steer_speed[1],
      ros_params_manager_.data().pid_lr_steer_speed[2],
      ros_params_manager_.data().pid_lr_steer_speed[3],
      ros_params_manager_.data().pid_lr_steer_speed[4]);
  fsm::Chassis::controller.rr_steer_speed = std::make_unique<PosPid>(
      ros_params_manager_.data().pid_rr_steer_speed[0], 
      ros_params_manager_.data().pid_rr_steer_speed[1],
      ros_params_manager_.data().pid_rr_steer_speed[2],
      ros_params_manager_.data().pid_rr_steer_speed[3],
      ros_params_manager_.data().pid_rr_steer_speed[4]);
  fsm::Chassis::controller.lf_steer_position = std::make_unique<PosRingPid>(
      ros_params_manager_.data().pid_lf_steer_position[0], 
      ros_params_manager_.data().pid_lf_steer_position[1],
      ros_params_manager_.data().pid_lf_steer_position[2],
      ros_params_manager_.data().pid_lf_steer_position[3],
      ros_params_manager_.data().pid_lf_steer_position[4],
      2 * M_PI);
  fsm::Chassis::controller.rf_steer_position = std::make_unique<PosRingPid>(
      ros_params_manager_.data().pid_rf_steer_position[0], 
      ros_params_manager_.data().pid_rf_steer_position[1],
      ros_params_manager_.data().pid_rf_steer_position[2],
      ros_params_manager_.data().pid_rf_steer_position[3],
      ros_params_manager_.data().pid_rf_steer_position[4],
      2 * M_PI);
  fsm::Chassis::controller.lr_steer_position = std::make_unique<PosRingPid>(
      ros_params_manager_.data().pid_lr_steer_position[0], 
      ros_params_manager_.data().pid_lr_steer_position[1],
      ros_params_manager_.data().pid_lr_steer_position[2],
      ros_params_manager_.data().pid_lr_steer_position[3],
      ros_params_manager_.data().pid_lr_steer_position[4],
      2 * M_PI);
  fsm::Chassis::controller.rr_steer_position = std::make_unique<PosRingPid>(
      ros_params_manager_.data().pid_rr_steer_position[0], 
      ros_params_manager_.data().pid_rr_steer_position[1],
      ros_params_manager_.data().pid_rr_steer_position[2],
      ros_params_manager_.data().pid_rr_steer_position[3],
      ros_params_manager_.data().pid_rr_steer_position[4],
      2 * M_PI);
  fsm::Chassis::controller.chassis_orientation_position = std::make_unique<PosRingPid>(
      ros_params_manager_.data().pid_chassis_orientation_position[0], 
      ros_params_manager_.data().pid_chassis_orientation_position[1],
      ros_params_manager_.data().pid_chassis_orientation_position[2],
      ros_params_manager_.data().pid_chassis_orientation_position[3],
      ros_params_manager_.data().pid_chassis_orientation_position[4],
      2 * M_PI);
  // clang-format on

  chassis_can->Begin();  // 开启can总线
  arm_can->Begin();
  fsm::fsm_list::start();  // 状态机开始监听事件
  info("fsm init ok");
}

void FluoriteChassis::Navigation() { info("navigation ok"); }

/**
 * @brief 开始接收所有链路的用户输入：sbus、图传链路键鼠、自定义控制器和调试键盘节点（也就是初始化一堆subscriber）
 */
void FluoriteChassis::UserInput() {
  sbus_sub_ = create_subscription<sbus_interface::msg::Sbus>(
      "/sbus", rclcpp::SensorDataQoS(), [&](const sbus_interface::msg::Sbus::SharedPtr msg) {
        if (msg->failsafe) {
          fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::NoForce>{});
          return;
        }

        control_msg_.left_x = -(double)msg->mapped_channels[3] / 1000.0;
        control_msg_.left_y = -(double)msg->mapped_channels[1] / 1000.0;
        control_msg_.right_x = (double)msg->mapped_channels[0] / 1000.0;
        control_msg_.right_y = (double)msg->mapped_channels[2] / 1000.0;
        control_msg_.ld = -msg->mapped_channels[4] / 1000.0;
        control_msg_.rd = -msg->mapped_channels[5] / 1000.0;
        control_msg_.ls = -msg->mapped_channels[6] / 1000.0;
        control_msg_.rs = -msg->mapped_channels[7] / 1000.0;
        control_msg_.sa = -msg->mapped_channels[8] / 1000;
        control_msg_.sb = -msg->mapped_channels[9] / 1000;
        control_msg_.sc = -msg->mapped_channels[10] / 1000;
        control_msg_.sd = -msg->mapped_channels[11] / 1000;
        control_msg_.se = -msg->mapped_channels[12] / 1000;
        control_msg_.sf = -msg->mapped_channels[13] / 1000;
        control_msg_.sg = -msg->mapped_channels[14] / 1000;
        control_msg_.sh = -msg->mapped_channels[15] / 1000;
      });
  debug_keyboard_keydown_sub_ = create_subscription<keyboard_msgs::msg::Key>(
      "/keydown", rclcpp::QoS(10).reliable(), [&](const keyboard_msgs::msg::Key::SharedPtr msg) {
        if (control_msg_.keyboard_key.find(msg->code) != control_msg_.keyboard_key.end()) {
          control_msg_.keyboard_key[msg->code] = true;
        }
        control_msg_.keyboard_modifier |= msg->modifiers;
      });
  debug_keyboard_keyup_sub_ = create_subscription<keyboard_msgs::msg::Key>(
      "/keyup", rclcpp::QoS(10).reliable(), [&](const keyboard_msgs::msg::Key::SharedPtr msg) {
        if (control_msg_.keyboard_key.find(msg->code) != control_msg_.keyboard_key.end()) {
          control_msg_.keyboard_key[msg->code] = false;
        }
        control_msg_.keyboard_modifier &= (~msg->modifiers);
      });
  fpv_link_input_sub_ = create_subscription<rm2_referee_msgs::msg::RemoteControl>(
      "/rm2_referee/remote_control", rclcpp::SensorDataQoS(), [&](const rm2_referee_msgs::msg::RemoteControl msg) {
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_W] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_W;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_S] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_S;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_A] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_A;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_D] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_D;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::MODIFIER_LSHIFT] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_LSHIFT;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::MODIFIER_LCTRL] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_LCTRL;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_Q] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_Q;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_E] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_E;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_R] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_R;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_F] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_F;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_G] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_G;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_Z] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_Z;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_X] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_X;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_C] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_C;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_V] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_V;
        control_msg_.keyboard_key[keyboard_msgs::msg::Key::KEY_B] =
            msg.keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_B;
      });

  info("user input subscribers spawned \033[32mmok\033[0m");
}

void FluoriteChassis::Referee() { info("referee ok"); }

/**
 * @brief 字面意思
 */
void FluoriteChassis::Blackbox() {
  blackbox_state_pub_ =
      this->create_publisher<fluorite_msgs::msg::ChassisState>("blackbox/chassis", rclcpp::SystemDefaultsQoS());
  blackbox_timer_ = this->create_wall_timer(10ms, [this] { blackbox_state_pub_->publish(fsm::Chassis::state_msg); });
  info("blackbox ok");
}

/**
 * @brief 设备在线状态监控
 */
void FluoriteChassis::DeviceMonitor() {
  device_monitor_timer_ = this->create_wall_timer(300ms, [this] {
    // device_monitor_.Update();
    // if (!device_monitor_.all_device_ok()) {
    //   fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::NoForce>{});
    // }
  });
  info("device monitor ok");
}

/**
 * @brief 把电机对象里的保存的电机状态复制到控制事件里
 */
void FluoriteChassis::UpdateFsmState() {
  fsm::Chassis::state_msg.base_yaw_encoder = rm::modules::algorithm::utils::LoopConstrain(
      fsm::Chassis::actuator.base_yaw->pos_rad() - ros_params_manager_.data().base_yaw_encoder_zero_offset, 0.0,
      M_PI * 2.0);
  fsm::Chassis::state_msg.lf_wheel_speed = fsm::Chassis::actuator.lf_wheel->rpm();
  fsm::Chassis::state_msg.rf_wheel_speed = fsm::Chassis::actuator.rf_wheel->rpm();
  fsm::Chassis::state_msg.lr_wheel_speed = fsm::Chassis::actuator.lr_wheel->rpm();
  fsm::Chassis::state_msg.rr_wheel_speed = fsm::Chassis::actuator.rr_wheel->rpm();
  fsm::Chassis::state_msg.lf_steer_position = rm::modules::algorithm::utils::LoopConstrain(
      fsm::Chassis::actuator.lf_steer->pos_rad() - ros_params_manager_.data().lf_steer_front_encoder, 0, 2 * M_PI);
  fsm::Chassis::state_msg.lr_steer_position = rm::modules::algorithm::utils::LoopConstrain(
      fsm::Chassis::actuator.lr_steer->pos_rad() - ros_params_manager_.data().lr_steer_front_encoder, 0, 2 * M_PI);
  fsm::Chassis::state_msg.rf_steer_position = rm::modules::algorithm::utils::LoopConstrain(
      fsm::Chassis::actuator.rf_steer->pos_rad() - ros_params_manager_.data().rf_steer_front_encoder, 0, 2 * M_PI);
  fsm::Chassis::state_msg.rr_steer_position = rm::modules::algorithm::utils::LoopConstrain(
      fsm::Chassis::actuator.rr_steer->pos_rad() - ros_params_manager_.data().rr_steer_front_encoder, 0, 2 * M_PI);
  fsm::Chassis::state_msg.lf_steer_speed = fsm::Chassis::actuator.lf_steer->rpm();
  fsm::Chassis::state_msg.rf_steer_speed = fsm::Chassis::actuator.rf_steer->rpm();
  fsm::Chassis::state_msg.lr_steer_speed = fsm::Chassis::actuator.lr_steer->rpm();
  fsm::Chassis::state_msg.rr_steer_speed = fsm::Chassis::actuator.rr_steer->rpm();
}

void FluoriteChassis::ParamHotUpdate() {}

}  // namespace fluorite::chassis
