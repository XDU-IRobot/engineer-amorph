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
#include "librm/modules/algorithm/utils.hpp"

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
  auto chassis_can = new hal::Can(ros_params_manager_.data().chassis_can_port.c_str());

  // clang-format off
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
 
  fsm::Chassis::controller.lf_wheel_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_lf_wheel_speed[0], 
      ros_params_manager_.data().pid_lf_wheel_speed[1],
      ros_params_manager_.data().pid_lf_wheel_speed[2],
      ros_params_manager_.data().pid_lf_wheel_speed[3],
      ros_params_manager_.data().pid_lf_wheel_speed[4]);
  fsm::Chassis::controller.rf_wheel_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_rf_wheel_speed[0], 
      ros_params_manager_.data().pid_rf_wheel_speed[1],
      ros_params_manager_.data().pid_rf_wheel_speed[2],
      ros_params_manager_.data().pid_rf_wheel_speed[3],
      ros_params_manager_.data().pid_rf_wheel_speed[4]);
  fsm::Chassis::controller.lr_wheel_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_lr_wheel_speed[0], 
      ros_params_manager_.data().pid_lr_wheel_speed[1],
      ros_params_manager_.data().pid_lr_wheel_speed[2],
      ros_params_manager_.data().pid_lr_wheel_speed[3],
      ros_params_manager_.data().pid_lr_wheel_speed[4]);
  fsm::Chassis::controller.rr_wheel_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_rr_wheel_speed[0], 
      ros_params_manager_.data().pid_rr_wheel_speed[1],
      ros_params_manager_.data().pid_rr_wheel_speed[2],
      ros_params_manager_.data().pid_rr_wheel_speed[3],
      ros_params_manager_.data().pid_rr_wheel_speed[4]);
  fsm::Chassis::controller.lf_steer_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_lf_steer_speed[0], 
      ros_params_manager_.data().pid_lf_steer_speed[1],
      ros_params_manager_.data().pid_lf_steer_speed[2],
      ros_params_manager_.data().pid_lf_steer_speed[3],
      ros_params_manager_.data().pid_lf_steer_speed[4]);
  fsm::Chassis::controller.rf_steer_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_rf_steer_speed[0], 
      ros_params_manager_.data().pid_rf_steer_speed[1],
      ros_params_manager_.data().pid_rf_steer_speed[2],
      ros_params_manager_.data().pid_rf_steer_speed[3],
      ros_params_manager_.data().pid_rf_steer_speed[4]);
  fsm::Chassis::controller.lr_steer_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_lr_steer_speed[0], 
      ros_params_manager_.data().pid_lr_steer_speed[1],
      ros_params_manager_.data().pid_lr_steer_speed[2],
      ros_params_manager_.data().pid_lr_steer_speed[3],
      ros_params_manager_.data().pid_lr_steer_speed[4]);
  fsm::Chassis::controller.rr_steer_speed = std::make_unique<rm::modules::algorithm::PID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_rr_steer_speed[0], 
      ros_params_manager_.data().pid_rr_steer_speed[1],
      ros_params_manager_.data().pid_rr_steer_speed[2],
      ros_params_manager_.data().pid_rr_steer_speed[3],
      ros_params_manager_.data().pid_rr_steer_speed[4]);
  fsm::Chassis::controller.lf_steer_position = std::make_unique<rm::modules::algorithm::RingPID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_lf_steer_position[0], 
      ros_params_manager_.data().pid_lf_steer_position[1],
      ros_params_manager_.data().pid_lf_steer_position[2],
      ros_params_manager_.data().pid_lf_steer_position[3],
      ros_params_manager_.data().pid_lf_steer_position[4],
      2 * M_PI);
  fsm::Chassis::controller.rf_steer_position = std::make_unique<rm::modules::algorithm::RingPID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_rf_steer_position[0], 
      ros_params_manager_.data().pid_rf_steer_position[1],
      ros_params_manager_.data().pid_rf_steer_position[2],
      ros_params_manager_.data().pid_rf_steer_position[3],
      ros_params_manager_.data().pid_rf_steer_position[4],
      2 * M_PI);
  fsm::Chassis::controller.lr_steer_position = std::make_unique<rm::modules::algorithm::RingPID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_lr_steer_position[0], 
      ros_params_manager_.data().pid_lr_steer_position[1],
      ros_params_manager_.data().pid_lr_steer_position[2],
      ros_params_manager_.data().pid_lr_steer_position[3],
      ros_params_manager_.data().pid_lr_steer_position[4],
      2 * M_PI);
  fsm::Chassis::controller.rr_steer_position = std::make_unique<rm::modules::algorithm::RingPID>(
      rm::modules::algorithm::PIDType::kPosition,
      ros_params_manager_.data().pid_rr_steer_position[0], 
      ros_params_manager_.data().pid_rr_steer_position[1],
      ros_params_manager_.data().pid_rr_steer_position[2],
      ros_params_manager_.data().pid_rr_steer_position[3],
      ros_params_manager_.data().pid_rr_steer_position[4],
      2 * M_PI);
  // clang-format on

  chassis_can->SetFilter(0, 0);  // 设置can过滤器
  chassis_can->Begin();          // 开启can总线
  fsm::fsm_list::start();        // 状态机开始监听事件
  info("fsm init ok");
}

void FluoriteChassis::Navigation() { info("navigation ok"); }

void FluoriteChassis::Remote() {
  static std::vector<short> last_channels(16, 0);
  remote_sub_ = this->create_subscription<sbus_interface::msg::Sbus>(
      "/sbus", rclcpp::SensorDataQoS(), [&](const sbus_interface::msg::Sbus::SharedPtr msg) {
        // remote_device_.IAmAlive();
        if (msg->failsafe) {
          fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::NoForce>{});
          return;
        }
        if (msg->mapped_channels[8] == -1000) {
          if ((last_channels[8] != -1000)) {  // SA up
            fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::FollowGimbal>{});
          }
        } else {
          fsm::DispatchEvent(event::chassis::SwitchMode<state::chassis::NoForce>{});
        }

        control_msg_.vx = msg->mapped_channels[1] * ros_params_manager_.data().control_sensitivity_vertical;
        control_msg_.vy = msg->mapped_channels[3] * ros_params_manager_.data().control_sensitivity_horizontal;
        control_msg_.vw = -msg->mapped_channels[0] * ros_params_manager_.data().control_sensitivity_horizontal;
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
      this->create_publisher<fluorite_msgs::msg::ChassisState>("blackbox/chassis", rclcpp::SystemDefaultsQoS());
  blackbox_timer_ = this->create_wall_timer(10ms, [this] { blackbox_state_pub_->publish(fsm::Chassis::state); });
  info("blackbox ok");
}

/**
 * @brief 设备在线状态监控
 * @note  因为irobotec还没有推进到这里所以暂时弄了一种很蹩脚的方法，只监控遥控器和imu
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
  fsm::Chassis::state.lf_wheel_speed = fsm::Chassis::actuator.lf_wheel->rpm();
  fsm::Chassis::state.rf_wheel_speed = fsm::Chassis::actuator.rf_wheel->rpm();
  fsm::Chassis::state.lr_wheel_speed = fsm::Chassis::actuator.lr_wheel->rpm();
  fsm::Chassis::state.rr_wheel_speed = fsm::Chassis::actuator.rr_wheel->rpm();
  fsm::Chassis::state.lf_steer_position = rm::modules::algorithm::utils::LoopConstrain(
      fsm::Chassis::actuator.lf_steer->pos_rad() - ros_params_manager_.data().lf_steer_front_encoder, 0, 2 * M_PI);
  fsm::Chassis::state.lr_steer_position = rm::modules::algorithm::utils::LoopConstrain(
      fsm::Chassis::actuator.lr_steer->pos_rad() - ros_params_manager_.data().lr_steer_front_encoder, 0, 2 * M_PI);
  fsm::Chassis::state.rf_steer_position = rm::modules::algorithm::utils::LoopConstrain(
      fsm::Chassis::actuator.rf_steer->pos_rad() - ros_params_manager_.data().rf_steer_front_encoder, 0, 2 * M_PI);
  fsm::Chassis::state.rr_steer_position = rm::modules::algorithm::utils::LoopConstrain(
      fsm::Chassis::actuator.rr_steer->pos_rad() - ros_params_manager_.data().rr_steer_front_encoder, 0, 2 * M_PI);
  fsm::Chassis::state.lf_steer_speed = fsm::Chassis::actuator.lf_steer->rpm();
  fsm::Chassis::state.rf_steer_speed = fsm::Chassis::actuator.rf_steer->rpm();
  fsm::Chassis::state.lr_steer_speed = fsm::Chassis::actuator.lr_steer->rpm();
  fsm::Chassis::state.rr_steer_speed = fsm::Chassis::actuator.rr_steer->rpm();
}

void FluoriteChassis::ParamHotUpdate() {}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteChassis>());
  rclcpp::shutdown();
  return 0;
}