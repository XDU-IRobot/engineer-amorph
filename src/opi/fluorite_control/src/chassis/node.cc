/**
 * @file  fluorite_chassis.cc
 * @brief 控制节点
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

using namespace std::chrono_literals;
using namespace rm;
using namespace rm::modules::algorithm::utils;

FluoriteChassis::FluoriteChassis(const rclcpp::NodeOptions &options)
    : rclcpp::Node("fluorite_control_chassis", options) {
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

FluoriteChassis::~FluoriteChassis() { fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::NoForce>{}); }

void FluoriteChassis::InitFsm() {
  auto chassis_can = new hal::Can(ros_params_manager_.data().can_port.c_str());

  // clang-format off
  fsm::Chassis::actuator.l_front_wheel = std::make_unique<device::M3508>(
      *chassis_can, 
      ros_params_manager_.data().l_front_wheel_id, 
      ros_params_manager_.data().l_front_wheel_reverse);
  fsm::Chassis::actuator.l_mid_wheel = std::make_unique<device::M3508>(
      *chassis_can,
      ros_params_manager_.data().l_mid_wheel_id,
      ros_params_manager_.data().l_mid_wheel_reverse);
  fsm::Chassis::actuator.l_rear_wheel = std::make_unique<device::M3508>(
      *chassis_can,
      ros_params_manager_.data().l_rear_wheel_id,
      ros_params_manager_.data().l_rear_wheel_reverse);
  fsm::Chassis::actuator.l_lift = std::make_unique<device::M3508>(
      *chassis_can,
      ros_params_manager_.data().l_lift_wheel_id,
      ros_params_manager_.data().l_lift_wheel_reverse);
  fsm::Chassis::actuator.r_front_wheel = std::make_unique<device::M3508>(
      *chassis_can,
      ros_params_manager_.data().r_front_wheel_id,
      ros_params_manager_.data().r_front_wheel_reverse);
  fsm::Chassis::actuator.r_mid_wheel = std::make_unique<device::M3508>(
      *chassis_can,
      ros_params_manager_.data().r_mid_wheel_id,
      ros_params_manager_.data().r_mid_wheel_reverse);
  fsm::Chassis::actuator.r_rear_wheel = std::make_unique<device::M3508>(
      *chassis_can,
      ros_params_manager_.data().r_rear_wheel_id,
      ros_params_manager_.data().r_rear_wheel_reverse);
  fsm::Chassis::actuator.r_lift = std::make_unique<device::M3508>(
      *chassis_can,
      ros_params_manager_.data().r_lift_wheel_id,
      ros_params_manager_.data().r_lift_wheel_reverse);
  
  fsm::Chassis::controller.l_front_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_l_front_speed[0], 
      ros_params_manager_.data().pid_l_front_speed[1],
      ros_params_manager_.data().pid_l_front_speed[2],
      12000, 0);
  fsm::Chassis::controller.l_mid_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_l_mid_speed[0], 
      ros_params_manager_.data().pid_l_mid_speed[1],
      ros_params_manager_.data().pid_l_mid_speed[2],
      12000, 0);
  fsm::Chassis::controller.l_rear_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_l_rear_speed[0], 
      ros_params_manager_.data().pid_l_rear_speed[1],
      ros_params_manager_.data().pid_l_rear_speed[2],
      12000, 0);
  fsm::Chassis::controller.l_lift_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_l_lift_speed[0], 
      ros_params_manager_.data().pid_l_lift_speed[1],
      ros_params_manager_.data().pid_l_lift_speed[2],
      12000, 0);
  fsm::Chassis::controller.r_front_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_r_front_speed[0], 
      ros_params_manager_.data().pid_r_front_speed[1],
      ros_params_manager_.data().pid_r_front_speed[2],
      12000, 0);
  fsm::Chassis::controller.r_mid_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_r_mid_speed[0], 
      ros_params_manager_.data().pid_r_mid_speed[1],
      ros_params_manager_.data().pid_r_mid_speed[2],
      12000, 0);
  fsm::Chassis::controller.r_rear_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_r_rear_speed[0], 
      ros_params_manager_.data().pid_r_rear_speed[1],
      ros_params_manager_.data().pid_r_rear_speed[2],
      12000, 0);
  fsm::Chassis::controller.r_lift_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_r_lift_speed[0], 
      ros_params_manager_.data().pid_r_lift_speed[1],
      ros_params_manager_.data().pid_r_lift_speed[2],
      12000, 0);
  fsm::Chassis::controller.l_lift_position = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_l_lift_position[0], 
      ros_params_manager_.data().pid_l_lift_position[1],
      ros_params_manager_.data().pid_l_lift_position[2],
      1000, 0);
  fsm::Chassis::controller.r_lift_position = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_r_lift_position[0], 
      ros_params_manager_.data().pid_r_lift_position[1],
      ros_params_manager_.data().pid_r_lift_position[2],
      1000, 0);
  // clang-format on

  chassis_can->SetFilter(0, 0);  // 设置can过滤器
  chassis_can->Begin();          // 开启can总线
  fsm::fsm_list::start();        // 状态机开始监听事件
  info("fsm init ok");
}

void FluoriteChassis::Navigation() { info("navigation ok"); }

void FluoriteChassis::Remote() {
  static int last_left_switch = 0;
  static int last_right_switch = 0;
  remote_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/rm2_remote/joy", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::Joy::SharedPtr msg) {
        remote_device_.IAmAlive();
        if (msg->buttons[0] == 3) {  // right at the middle
          if ((last_left_switch != 3) && (msg->buttons[1] == 3)) {
            fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::Neutral>{});
          } else if ((last_left_switch != 2) && (msg->buttons[1] == 2)) {
            fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::Lower>{});
          } else if ((last_left_switch != 1) && (msg->buttons[1] == 1)) {
            fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::Lift>{});
          }
        } else {
          fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::NoForce>{});
        }
        last_left_switch = msg->buttons[1];
        last_right_switch = msg->buttons[0];

        control_msg_.vx = msg->axes[0] * ros_params_manager_.data().control_sensitivity_vertical;
        control_msg_.vy = msg->axes[1] * ros_params_manager_.data().control_sensitivity_horizontal;
        control_msg_.vw = -msg->axes[4] * ros_params_manager_.data().control_sensitivity_horizontal;
        if (ros_params_manager_.data().control_invert_vertical) {
          control_msg_.vx = -control_msg_.vx;
        }
        if (ros_params_manager_.data().control_invert_horizontal) {
          control_msg_.vy = -control_msg_.vy;
          control_msg_.vw = -control_msg_.vw;
        }
      });
  info("remote ok");
}

void FluoriteChassis::Referee() { info("referee ok"); }

/**
 * @brief 字面意思
 */
void FluoriteChassis::Blackbox() {
  blackbox_state_pub_ =
      this->create_publisher<fluorite_msgs::msg::ChassisState>("blackbox/state", rclcpp::SystemDefaultsQoS());
  blackbox_timer_ = this->create_wall_timer(10ms, [this] { blackbox_state_pub_->publish(fsm::Chassis::state); });
  info("blackbox ok");
}

/**
 * @brief 设备在线状态监控
 * @note  因为irobotec还没有推进到这里所以暂时弄了一种很蹩脚的方法，只监控遥控器和imu
 */
void FluoriteChassis::DeviceMonitor() {
  device_monitor_ << remote_device_;
  device_monitor_timer_ = this->create_wall_timer(300ms, [this] {
    device_monitor_.Update();
    if (!device_monitor_.all_device_ok()) {
      fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::NoForce>{});
    }
  });
  info("device monitor ok");
}

/**
 * @brief 把电机对象里的保存的电机状态复制到控制事件里
 */
void FluoriteChassis::UpdateFsmState() {
  fsm::Chassis::state.l_front_speed = fsm::Chassis::actuator.l_front_wheel->rpm();
  fsm::Chassis::state.l_mid_speed = fsm::Chassis::actuator.l_mid_wheel->rpm();
  fsm::Chassis::state.l_rear_speed = fsm::Chassis::actuator.l_rear_wheel->rpm();
  fsm::Chassis::state.l_lift_speed = fsm::Chassis::actuator.l_lift->rpm();
  fsm::Chassis::state.r_front_speed = fsm::Chassis::actuator.r_front_wheel->rpm();
  fsm::Chassis::state.r_mid_speed = fsm::Chassis::actuator.r_mid_wheel->rpm();
  fsm::Chassis::state.r_rear_speed = fsm::Chassis::actuator.r_rear_wheel->rpm();
  fsm::Chassis::state.r_lift_speed = fsm::Chassis::actuator.r_lift->rpm();
  fsm::Chassis::controller.l_lift_encoder.Update(fsm::Chassis::actuator.l_lift->encoder());
  fsm::Chassis::state.l_lift_position = fsm::Chassis::controller.l_lift_encoder.value();
  fsm::Chassis::controller.r_lift_encoder.Update(fsm::Chassis::actuator.r_lift->encoder());
  fsm::Chassis::state.r_lift_position = fsm::Chassis::controller.r_lift_encoder.value();
}

void FluoriteChassis::ParamHotUpdate() {}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteChassis>());
  rclcpp::shutdown();
  return 0;
}