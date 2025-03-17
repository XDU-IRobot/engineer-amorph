/**
 * @file  fluorite_chassis.cc
 * @brief 控制节点
 */

#include "arm/node.hpp"

#include <chrono>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <fo_lpf.h>
#include <librm.hpp>
#include <tinyfsm.hpp>
#include "librm/modules/algorithm/ahrs/ahrs_interface.h"
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
  InitializeShit();        // 初始化状态机
  DeviceMonitor();         // 设备在线状态监控
  Ins();                   // 启动IMU接收
  Remote();                // 启动遥控器接收
  Keyboard();              // 启动键盘接收
  Referee();               // 启动裁判系统接收
  Blackbox();              // 启动黑匣子（广播state和control）
  JointStatesPublisher();  // 启动关节状态发布
  TFBroadcaster();         // 启动imu到base_yaw_link的tf广播

  // 启动控制循环
  info("control node initialize ok");
  control_loop_timer_ = create_wall_timer(2.5ms, [this] {
    ProcessSensorData();
    ParamHotUpdate();
    fluorite::fsm::DispatchEvent(control_msg_);
    rm::device::DjiMotor<>::SendCommand();
    rm::device::DirectDriveMotor::SendCommand();
  });
  info("CONTROL UP");
}

FluoriteArm::~FluoriteArm() {
  using namespace fluorite::fsm;
  DispatchEvent(event::SwitchMode<state::arm::NoForce>{});
  DispatchEvent(event::SwitchMode<state::gimbal_base::NoForce>{});
}

/**
 * @brief 因为一些对象创建时要根据yaml文件中的参数初始化，所以很多对象不能原地构造，只能把那些初始化全部堆在这里
 */
void FluoriteArm::InitializeShit() {
  using Ddm = rm::device::DirectDriveMotor;
  using DmMit = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;
  using DmSettingsMit = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;
  using PosPID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  robot_model_loader::RobotModelLoader robot_model_loader{rclcpp::Node::SharedPtr{this}};
  const auto kinematic_model = robot_model_loader.getModel();
  auto robot_state = std::make_shared<moveit::core::RobotState>(kinematic_model);

  // 这个can对象的生命周期和程序一样长，所以new一个就不管他了，不用delete
  auto arm_can = new rm::hal::Can(ros_params_manager_.data().arm_can_port.c_str());
  fluorite::controller = std::make_unique<fluorite::Controller>(
      PidTrajectoryTracker<6>{
          PosPID{static_cast<float>(ros_params_manager_.data().pid_yaw_speed[0]),
                 static_cast<float>(ros_params_manager_.data().pid_yaw_speed[1]),
                 static_cast<float>(ros_params_manager_.data().pid_yaw_speed[2]),
                 static_cast<float>(ros_params_manager_.data().pid_yaw_speed[3]),
                 static_cast<float>(ros_params_manager_.data().pid_yaw_speed[4])},  // yaw 速度环
          PosPID{static_cast<float>(ros_params_manager_.data().pid_yaw_position[0]),
                 static_cast<float>(ros_params_manager_.data().pid_yaw_position[1]),
                 static_cast<float>(ros_params_manager_.data().pid_yaw_position[2]),
                 static_cast<float>(ros_params_manager_.data().pid_yaw_position[3]),
                 static_cast<float>(ros_params_manager_.data().pid_yaw_position[4])},  // yaw 位置环
          PosPID{static_cast<float>(ros_params_manager_.data().pid_pitch_1_speed[0]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_1_speed[1]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_1_speed[2]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_1_speed[3]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_1_speed[4])},  // pitch_1 速度环
          PosPID{static_cast<float>(ros_params_manager_.data().pid_pitch_1_position[0]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_1_position[1]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_1_position[2]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_1_position[3]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_1_position[4])},  // pitch_1 位置环
          PosPID{static_cast<float>(ros_params_manager_.data().pid_pitch_2_speed[0]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_2_speed[1]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_2_speed[2]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_2_speed[3]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_2_speed[4])},  // pitch_1 速度环
          PosPID{static_cast<float>(ros_params_manager_.data().pid_pitch_2_position[0]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_2_position[1]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_2_position[2]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_2_position[3]),
                 static_cast<float>(ros_params_manager_.data().pid_pitch_2_position[4])},  // pitch_1 位置环
          PosPID{static_cast<float>(ros_params_manager_.data().pid_end_yaw_1_speed[0]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_1_speed[1]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_1_speed[2]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_1_speed[3]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_1_speed[4])},  // end_yaw_1 速度环
          PosPID{static_cast<float>(ros_params_manager_.data().pid_end_yaw_1_position[0]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_1_position[1]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_1_position[2]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_1_position[3]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_1_position[4])},  // end_yaw_1 位置环
          PosPID{static_cast<float>(ros_params_manager_.data().pid_end_pitch_speed[0]),
                 static_cast<float>(ros_params_manager_.data().pid_end_pitch_speed[1]),
                 static_cast<float>(ros_params_manager_.data().pid_end_pitch_speed[2]),
                 static_cast<float>(ros_params_manager_.data().pid_end_pitch_speed[3]),
                 static_cast<float>(ros_params_manager_.data().pid_end_pitch_speed[4])},  // end_pitch 速度环
          PosPID{static_cast<float>(ros_params_manager_.data().pid_end_pitch_position[0]),
                 static_cast<float>(ros_params_manager_.data().pid_end_pitch_position[1]),
                 static_cast<float>(ros_params_manager_.data().pid_end_pitch_position[2]),
                 static_cast<float>(ros_params_manager_.data().pid_end_pitch_position[3]),
                 static_cast<float>(ros_params_manager_.data().pid_end_pitch_position[4])},  // end_pitch 位置环
          PosPID{static_cast<float>(ros_params_manager_.data().pid_end_yaw_2_speed[0]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_2_speed[1]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_2_speed[2]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_2_speed[3]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_2_speed[4])},  // end_yaw_2 速度环
          PosPID{static_cast<float>(ros_params_manager_.data().pid_end_yaw_2_position[0]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_2_position[1]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_2_position[2]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_2_position[3]),
                 static_cast<float>(ros_params_manager_.data().pid_end_yaw_2_position[4])},  // end_yaw_2 位置环
      },
      PosPID{static_cast<float>(ros_params_manager_.data().pid_base_yaw_speed[0]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_speed[1]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_speed[2]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_speed[3]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_speed[4])},  // base_yaw 速度环
      PosPID{static_cast<float>(ros_params_manager_.data().pid_base_yaw_position[0]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_position[1]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_position[2]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_position[3]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_position[4])},  // base_yaw 位置环
      std::move(robot_state), moveit::planning_interface::MoveGroupInterface{rclcpp::Node::SharedPtr{this}, "arm"});
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

  arm_can->Begin();                  // 启动can总线接收
  fluorite::fsm::fsm_list::start();  // 状态机开始监听事件
  info("objects construction ok");
}

void FluoriteArm::Ins() {
  ins_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
        mahony_.Update(rm::modules::algorithm::ImuData6Dof{
            (float)msg->angular_velocity.y / 360 * 2 * M_PIf, (float)msg->angular_velocity.z / 360 * 2 * M_PIf,
            (float)msg->angular_velocity.x / 360 * 2 * M_PIf, (float)msg->linear_acceleration.y,
            (float)msg->linear_acceleration.z, (float)msg->linear_acceleration.x});
        fluorite::state.imu_yaw = mahony_.euler_angle().roll;
        fluorite::state.imu_pitch = mahony_.euler_angle().yaw;
        fluorite::state.imu_roll = mahony_.euler_angle().pitch;
      });
  info("ins ok");
}

void FluoriteArm::Remote() {
  static std::array<short, 16> last_channels{};
  remote_sub_ = create_subscription<sbus_interface::msg::Sbus>(
      "/sbus", rclcpp::SensorDataQoS(), [&](const sbus_interface::msg::Sbus::SharedPtr msg) {
        // remote_device_.IAmAlive();
        control_msg_.left_x = -(double)msg->mapped_channels[3] / 1000.0;
        control_msg_.left_y = -(double)msg->mapped_channels[1] / 1000.0;
        control_msg_.right_x = (double)msg->mapped_channels[0] / 1000.0;
        control_msg_.right_y = (double)msg->mapped_channels[2] / 1000.0;
        control_msg_.sa = -msg->mapped_channels[8] / 1000;
        control_msg_.sb = -msg->mapped_channels[9] / 1000;
        control_msg_.sc = -msg->mapped_channels[10] / 1000;
        control_msg_.sd = -msg->mapped_channels[11] / 1000;
        control_msg_.se = -msg->mapped_channels[12] / 1000;
        control_msg_.sf = -msg->mapped_channels[13] / 1000;
        control_msg_.sg = -msg->mapped_channels[14] / 1000;
        control_msg_.sh = -msg->mapped_channels[15] / 1000;

        last_channels = msg->mapped_channels;
      });
  info("remote ok");
}

void FluoriteArm::Keyboard() {
  keyboard_keydown_sub_ = create_subscription<keyboard_msgs::msg::Key>(
      "/keydown", rclcpp::SensorDataQoS(), [&](const keyboard_msgs::msg::Key::SharedPtr msg) {
        if (control_msg_.keyboard_key.find(msg->code) != control_msg_.keyboard_key.end()) {
          control_msg_.keyboard_key[msg->code] = true;
        }
        control_msg_.keyboard_modifier |= msg->modifiers;
      });
  keyboard_keyup_sub_ = create_subscription<keyboard_msgs::msg::Key>(
      "/keyup", rclcpp::SensorDataQoS(), [&](const keyboard_msgs::msg::Key::SharedPtr msg) {
        if (control_msg_.keyboard_key.find(msg->code) != control_msg_.keyboard_key.end()) {
          control_msg_.keyboard_key[msg->code] = false;
        }
        control_msg_.keyboard_modifier &= (~msg->modifiers);
      });
  info("keyboard ok");
}

void FluoriteArm::Referee() { info("referee ok"); }

/**
 * @brief
 * 把fluorite::state广播出去，方便调试的时候看数据，没有实质性作用，不需要debug或者记录log的时候可以把这个timer关掉
 */
void FluoriteArm::Blackbox() {
  static auto PopulatePidMsg = [](fluorite_msgs::msg::Pid &msg, const fluorite::Controller::PosPID &pid) {
    msg.kp = pid.kp_;
    msg.ki = pid.ki_;
    msg.kd = pid.kd_;
    msg.i_max = pid.max_iout_;
    msg.output_max = pid.max_out_;
    msg.error = pid.last_error();
    msg.p_out = pid.p_out();
    msg.i_out = pid.i_out();
    msg.d_out = pid.d_out();
    msg.out = pid.value();
  };
  blackbox_state_pub_ =
      create_publisher<fluorite_msgs::msg::ArmState>("fluorite/blackbox/arm_state", rclcpp::SystemDefaultsQoS());
  blackbox_timer_ = create_wall_timer(20ms, [this] {
    fluorite::state.header.stamp = now();
    for (size_t i = 0; i < 12; ++i) {
      PopulatePidMsg(fluorite::state.pids[i], fluorite::controller->trajectory_tracker.pids()[i]);
    }
    blackbox_state_pub_->publish(fluorite::state);
  });
  info("blackbox ok");
}

/**
 * @brief 设备在线状态监控
 */
void FluoriteArm::DeviceMonitor() {
  // device_monitor_timer_ = create_wall_timer(300ms, [this] {
  //   // device_monitor_.Update();
  //   // if (!device_monitor_.all_device_ok()) {
  //   //   fsm::DispatchEvent(event::arm::SwitchMode<state::arm::NoForce>{});
  //   // }
  // });
  info("device monitor ok");
}

/**
 * @brief 从电机对象里拿数据，做单位换算、范围处理、滤波器处理等等，最后更新fluorite::state
 */
void FluoriteArm::ProcessSensorData() {
  fluorite::state.base_yaw_encoder =
      fluorite::actuator->base_yaw.pos_rad() - ros_params_manager_.data().base_yaw_encoder_zero_offset;
  fluorite::state.base_yaw_encoder_speed = fluorite::actuator->base_yaw.rpm();
  fluorite::state.yaw_encoder = fluorite::actuator->yaw.pos() - ros_params_manager_.data().yaw_encoder_zero_offset;
  fluorite::state.yaw_encoder_speed = fluorite::actuator->yaw.vel();
  fluorite::state.pitch_1_encoder =
      fluorite::actuator->pitch_1.pos() - ros_params_manager_.data().pitch_1_encoder_zero_offset;
  fluorite::state.pitch_1_encoder_speed = fluorite::actuator->pitch_1.vel();
  fluorite::state.pitch_2_encoder =
      fluorite::actuator->pitch_2.pos() - ros_params_manager_.data().pitch_2_encoder_zero_offset;
  fluorite::state.pitch_2_encoder_speed = fluorite::actuator->pitch_2.vel();
  fluorite::state.end_yaw_1_encoder =
      fluorite::actuator->end_yaw_1.pos() - ros_params_manager_.data().end_yaw_1_encoder_zero_offset;
  fluorite::state.end_yaw_1_encoder_speed = fluorite::actuator->end_yaw_1.vel();
  fluorite::state.end_pitch_encoder =
      fluorite::actuator->end_pitch.pos() - ros_params_manager_.data().end_pitch_encoder_zero_offset;
  fluorite::state.end_pitch_encoder_speed = fluorite::actuator->end_pitch.vel();
  fluorite::state.end_yaw_2_encoder =
      fluorite::actuator->end_yaw_2.pos_rad() - ros_params_manager_.data().end_yaw_2_encoder_zero_offset;
  fluorite::state.end_yaw_2_encoder_speed = fluorite::actuator->end_yaw_2.rpm();
}

/**
 * @brief 获取参数服务器中的最新参数，更新到pid对象里
 */
void FluoriteArm::ParamHotUpdate() {}

/**
 * @brief 把关节状态发布给joint_state_publisher节点，jsp节点再转发给moveit，频率50hz
 */
void FluoriteArm::JointStatesPublisher() {
  joint_states_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("fluorite/arm/joint_states", rclcpp::SystemDefaultsQoS());
  joint_states_msg_.name = {"yaw_joint",      "pitch1_joint",    "pitch2_joint",
                            "end_yaw1_joint", "end_pitch_joint", "end_yaw2_joint"};
  joint_states_msg_.position.resize(6);
  joint_states_msg_.velocity.resize(6);
  joint_states_pub_timer_ = create_wall_timer(20ms, [this] {
    joint_states_msg_.header.stamp = now();
    joint_states_msg_.position[0] = fluorite::state.yaw_encoder;
    joint_states_msg_.position[1] = fluorite::state.pitch_1_encoder;
    joint_states_msg_.position[2] = fluorite::state.pitch_2_encoder;
    joint_states_msg_.position[3] = fluorite::state.end_yaw_1_encoder;
    joint_states_msg_.position[4] = fluorite::state.end_pitch_encoder;
    joint_states_msg_.position[5] = fluorite::state.end_yaw_2_encoder;
    joint_states_msg_.velocity[0] = fluorite::state.yaw_encoder_speed;
    joint_states_msg_.velocity[1] = fluorite::state.pitch_1_encoder_speed;
    joint_states_msg_.velocity[2] = fluorite::state.pitch_2_encoder_speed;
    joint_states_msg_.velocity[3] = fluorite::state.end_yaw_1_encoder_speed;
    joint_states_msg_.velocity[4] = fluorite::state.end_pitch_encoder_speed;
    joint_states_msg_.velocity[5] = fluorite::state.end_yaw_2_encoder_speed;
    joint_states_pub_->publish(joint_states_msg_);
  });
  info("jsp ok");
}

/**
 * @brief 发布从imu坐标系(world)到base_yaw_link坐标系的tf，频率50hz
 */
void FluoriteArm::TFBroadcaster() {
  world_to_base_yaw_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  world_to_base_yaw_tf_.header.frame_id = "world";
  world_to_base_yaw_tf_.child_frame_id = "base_yaw_link";
  world_to_base_yaw_tf_broadcaster_timer_ = create_wall_timer(20ms, [this] {
    world_to_base_yaw_tf_.header.stamp = now();
    world_to_base_yaw_tf_quat_.setRPY(fluorite::state.imu_roll, fluorite::state.imu_pitch, fluorite::state.imu_yaw);
    world_to_base_yaw_tf_.transform.rotation.x = world_to_base_yaw_tf_quat_.x();
    world_to_base_yaw_tf_.transform.rotation.y = world_to_base_yaw_tf_quat_.y();
    world_to_base_yaw_tf_.transform.rotation.z = world_to_base_yaw_tf_quat_.z();
    world_to_base_yaw_tf_.transform.rotation.w = world_to_base_yaw_tf_quat_.w();
    world_to_base_yaw_tf_broadcaster_->sendTransform(world_to_base_yaw_tf_);
  });
  info("tf broadcaster ok");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteArm>());
  rclcpp::shutdown();
  return 0;
}