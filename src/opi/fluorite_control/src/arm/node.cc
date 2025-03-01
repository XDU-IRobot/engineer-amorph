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
#include <wiringPi.h>
#endif

#include "arm/fsm.hpp"
#include "common/logger.hpp"

using namespace std::chrono_literals;

FluoriteArm::FluoriteArm(const rclcpp::NodeOptions &options) : rclcpp::Node("fluorite_control_arm", options) {
  ros_params_manager_.Init();  // 读取yaml文件中的参数、启动参数服务器
#if __has_include("wiringPi.h")
  wiringPiSetup();  // 初始化wiringPi
#endif
  this->InitializeShit();  // 初始化状态机
  this->DeviceMonitor();   // 设备在线状态监控
  this->Remote();          // 启动遥控器接收
  this->Referee();         // 启动裁判系统接收
  this->Navigation();      // 启动和导航节点的通信
  this->Blackbox();        // 启动黑匣子（广播state和control）

  // 启动控制循环
  info("control node initialize ok");
  control_loop_timer_ = this->create_wall_timer(2.5ms, [this] {
    this->UpdateFsmState();
    this->ParamHotUpdate();
    fluorite::fsm::DispatchEvent(control_msg_);
    // device::DjiMotor<>::SendCommand();
  });
  info("CONTROL UP");
}

FluoriteArm::~FluoriteArm() {
  using namespace fluorite;
  fsm::DispatchEvent(fsm::event::SwitchMode<fsm::state::arm::NoForce>{});
  fsm::DispatchEvent(fsm::event::SwitchMode<fsm::state::gimbal_base::NoForce>{});
}

/**
 * @brief 因为一些对象创建时要根据yaml文件中的参数初始化，所以很多对象不能原地构造，只能把那些初始化全部堆在这里
 */
void FluoriteArm::InitializeShit() {
  using Ddm = rm::device::DirectDriveMotor;
  using DmMit = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;
  using DmSettingsMit = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;
  using PosPID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  // 这个can对象的生命周期和程序一样长，所以new一个就不管他了，不用delete
  auto arm_can = new rm::hal::Can(ros_params_manager_.data().arm_can_port.c_str());
  fluorite::controller = std::make_unique<fluorite::Controller>(PidTrajectoryTracker<6>{
      PosPID{0, 0, 0, 0, 0},  // yaw 速度环
      PosPID{0, 0, 0, 0, 0},  // yaw 位置环
      PosPID{0, 0, 0, 0, 0},  // pitch_1 速度环
      PosPID{0, 0, 0, 0, 0},  // pitch_1 位置环
      PosPID{0, 0, 0, 0, 0},  // pitch_2 速度环
      PosPID{0, 0, 0, 0, 0},  // pitch_2 位置环
      PosPID{0, 0, 0, 0, 0},  // end_yaw_1 速度环
      PosPID{0, 0, 0, 0, 0},  // end_yaw_1 位置环
      PosPID{0, 0, 0, 0, 0},  // end_pitch 速度环
      PosPID{0, 0, 0, 0, 0},  // end_pitch 位置环
      PosPID{0, 0, 0, 0, 0},  // end_yaw 速度环
      PosPID{0, 0, 0, 0, 0}   // end_yaw 位置环
  });
  fluorite::actuator = std::make_unique<fluorite::Actuator>(
      Ddm{*arm_can, static_cast<size_t>(ros_params_manager_.data().base_yaw_id)},
      DmMit{*arm_can,
            DmSettingsMit{.master_id = ros_params_manager_.data().yaw_master_id,
                          .slave_id = ros_params_manager_.data().yaw_slave_id,
                          .p_max = 12.5f,
                          .v_max = 45.f,
                          .t_max = 54.f,
                          .kp_range = {0, 500},
                          .kd_range = {0, 50}},
            ros_params_manager_.data().yaw_reverse},
      DmMit{*arm_can,
            DmSettingsMit{.master_id = ros_params_manager_.data().pitch_1_master_id,
                          .slave_id = ros_params_manager_.data().pitch_1_slave_id,
                          .p_max = 12.5f,
                          .v_max = 45.f,
                          .t_max = 54.f,
                          .kp_range = {0, 500},
                          .kd_range = {0, 50}},
            ros_params_manager_.data().pitch_1_reverse},
      DmMit{*arm_can,
            DmSettingsMit{.master_id = ros_params_manager_.data().pitch_2_master_id,
                          .slave_id = ros_params_manager_.data().pitch_2_slave_id,
                          .p_max = 12.5f,
                          .v_max = 45.f,
                          .t_max = 54.f,
                          .kp_range = {0, 500},
                          .kd_range = {0, 50}},
            ros_params_manager_.data().pitch_2_reverse},
      DmMit{*arm_can,
            DmSettingsMit{.master_id = ros_params_manager_.data().end_yaw_1_master_id,
                          .slave_id = ros_params_manager_.data().end_yaw_1_slave_id,
                          .p_max = 12.5f,
                          .v_max = 30.f,
                          .t_max = 10.f,
                          .kp_range = {0, 500},
                          .kd_range = {0, 50}},
            ros_params_manager_.data().end_yaw_1_reverse},
      DmMit{*arm_can, DmSettingsMit{.master_id = 0xc7,
                                    .slave_id = 0xc8,
                                    .p_max = 12.5f,
                                    .v_max = 30.f,
                                    .t_max = 10.f,
                                    .kp_range = {0, 500},
                                    .kd_range = {0, 50}}},
      rm::device::GM6020{*arm_can, static_cast<uint16_t>(ros_params_manager_.data().end_yaw_2_id),
                         ros_params_manager_.data().end_yaw_2_reverse},
      SuckerSystem{Pin<OUTPUT>(ros_params_manager_.data().sucker_out_valve_pin),
                   Pin<OUTPUT>(ros_params_manager_.data().sucker_in_valve_pin),
                   Pin<PWM_OUTPUT>(ros_params_manager_.data().sucker_vaccum_pump_pin)},
      StorageSystem{Pin<OUTPUT>(ros_params_manager_.data().storage_left_valve_pin),
                    Pin<OUTPUT>(ros_params_manager_.data().storage_right_valve_pin),
                    Pin<PWM_OUTPUT>(ros_params_manager_.data().storage_vaccum_pump_pin)});

  arm_can->SetFilter(0, 0);          // 设置can过滤器
  arm_can->Begin();                  // 启动can总线接收
  fluorite::fsm::fsm_list::start();  // 状态机开始监听事件
  info("objects construction ok");
}

void FluoriteArm::Navigation() { info("navigation ok"); }

void FluoriteArm::Remote() {
  static std::array<short, 16> last_channels{};
  remote_sub_ = this->create_subscription<sbus_interface::msg::Sbus>(
      "/sbus", rclcpp::SensorDataQoS(), [&](const sbus_interface::msg::Sbus::SharedPtr msg) {
        // remote_device_.IAmAlive();

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
  blackbox_timer_ = this->create_wall_timer(20ms, [this] { blackbox_state_pub_->publish(fluorite::state); });
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