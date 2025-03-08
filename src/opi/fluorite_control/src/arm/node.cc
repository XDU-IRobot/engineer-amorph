/**
 * @file  fluorite_chassis.cc
 * @brief 控制节点
 */

#include "arm/node.hpp"

#include <chrono>

#include <fo_lpf.h>
#include <keyboard_msgs/msg/detail/key__struct.hpp>
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
  InitializeShit();        // 初始化状态机
  DeviceMonitor();         // 设备在线状态监控
  Ins();                   // 启动IMU接收
  Remote();                // 启动遥控器接收
  Keyboard();              // 启动键盘接收
  Referee();               // 启动裁判系统接收
  Blackbox();              // 启动黑匣子（广播state和control）
  JointStatesPublisher();  // 启动关节状态发布
  TFBroadcaster();         // 启动imu到base_yaw_link的tf广播
  TFListener();            // 启动tf监听，为了实时更新当前机械臂末端的位姿

  // 启动控制循环
  info("control node initialize ok");
  control_loop_timer_ = create_wall_timer(2.5ms, [this] {
    ProcessSensorData();
    ParamHotUpdate();
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
      moveit::planning_interface::MoveGroupInterface{rclcpp::Node::SharedPtr{this}, "arm"});
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
  auto quaternion_buffer = new float[4];  // 这里不用担心free的问题，因为这个数组的生命周期和程序一样长
  auto euler_angle_buffer = new float[3];
  ins_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS(),
                                                        [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
                                                          using rm::modules::algorithm::utils::QuatToEuler;
                                                          quaternion_buffer[0] = msg->orientation.w;
                                                          quaternion_buffer[1] = msg->orientation.x;
                                                          quaternion_buffer[2] = msg->orientation.y;
                                                          quaternion_buffer[3] = msg->orientation.z;
                                                          QuatToEuler(quaternion_buffer, euler_angle_buffer);
                                                          fluorite::state.imu_yaw = euler_angle_buffer[0];
                                                          fluorite::state.imu_pitch = euler_angle_buffer[1];
                                                          fluorite::state.imu_roll = euler_angle_buffer[2];
                                                        });
  info("ins ok");
}

void FluoriteArm::Remote() {
  static std::array<short, 16> last_channels{};
  remote_sub_ = create_subscription<sbus_interface::msg::Sbus>("/sbus", rclcpp::SensorDataQoS(),
                                                               [&](const sbus_interface::msg::Sbus::SharedPtr msg) {
                                                                 // remote_device_.IAmAlive();

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
 * 把fluorite::state广播出去，方便调试的时候看数据，没有实质性作用，确认不需要debug或者记录log的时候可以把这个timer关掉
 */
void FluoriteArm::Blackbox() {
  blackbox_state_pub_ =
      create_publisher<fluorite_msgs::msg::ArmState>("fluorite/blackbox/arm_state", rclcpp::SystemDefaultsQoS());
  blackbox_timer_ = create_wall_timer(20ms, [this] { blackbox_state_pub_->publish(fluorite::state); });
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
  fluorite::state.base_yaw_encoder = fluorite::actuator->base_yaw.encoder();
  fluorite::state.base_yaw_encoder_speed = fluorite::actuator->base_yaw.rpm();
  fluorite::state.yaw_encoder = fluorite::actuator->yaw.pos();
  fluorite::state.yaw_encoder_speed = fluorite::actuator->yaw.vel();
  fluorite::state.pitch_1_encoder = fluorite::actuator->pitch_1.pos();
  fluorite::state.pitch_1_encoder_speed = fluorite::actuator->pitch_1.vel();
  fluorite::state.pitch_2_encoder = fluorite::actuator->pitch_2.pos();
  fluorite::state.pitch_2_encoder_speed = fluorite::actuator->pitch_2.vel();
  fluorite::state.end_yaw_1_encoder = fluorite::actuator->end_yaw_1.pos();
  fluorite::state.end_yaw_1_encoder_speed = fluorite::actuator->end_yaw_1.vel();
  fluorite::state.end_pitch_encoder = fluorite::actuator->end_pitch.pos();
  fluorite::state.end_pitch_encoder_speed = fluorite::actuator->end_pitch.vel();
  fluorite::state.end_yaw_2_encoder = fluorite::actuator->end_yaw_2.pos_rad();
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
}

/**
 * @brief 监听tf，实时更新当前机械臂末端的位姿
 */
void FluoriteArm::TFListener() {
  tf_listener_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  end_effector_pose_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_listener_buffer_, this);
  end_effector_pose_tf_listener_timer_ = create_wall_timer(20ms, [this] {
    try {
      fluorite::state.end_effector_pose =
          tf_listener_buffer_->lookupTransform("end_yaw2_link", "base_yaw_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
    }
  });
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteArm>());
  rclcpp::shutdown();
  return 0;
}