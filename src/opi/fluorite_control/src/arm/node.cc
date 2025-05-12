/**
 * @file  arm/node.cc
 * @brief 大yaw轴基座+机械臂控制节点
 */

#include "arm/node.hpp"

#include <chrono>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
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
  InitializeShit();          // 初始化状态机
  DeviceMonitor();           // 设备在线状态监控
  Ins();                     // 启动IMU接收
  UserInput();               // 启动sbus、图传链路键鼠、自定义控制器和键盘调试输入接收
  Referee();                 // 启动裁判系统接收
  Blackbox();                // 发布调试信息
  JointStatesPublisher();    // 启动关节状态发布
  TFBroadcaster();           // 启动imu到base_yaw_link的tf广播
  BehaviourTreeInterface();  // 启动action和service服务器对接行为树节点

  // 启动控制循环
  info("all initialization finished \033[32mmok\033[0m");
  control_loop_timer_ = create_wall_timer(3.0ms, [this] {
    ProcessSensorData();
    ParamHotUpdate();
    fluorite::fsm::DispatchEvent(control_msg_);
    rm::device::DjiMotor<>::SendCommand();
    rm::device::DirectDriveMotor::SendCommand();
  });
  info("\033[32mm==========CONTROL IS UP!==========\033[0m");
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
  using PosRingPID = rm::modules::algorithm::RingPID<rm::modules::algorithm::PIDType::kPosition>;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 这个can对象的生命周期和程序一样长，所以new一个就不管他了，不用delete
  auto *arm_can = new rm::hal::Can(ros_params_manager_.data().arm_can_port.c_str());
  fluorite::controller = std::make_unique<fluorite::Controller>(
      PidTrajectoryTracker<6>{
          // pid controllers
          {
              std::make_shared<PosPID>(
                  ros_params_manager_.data().pid_yaw_speed[0], ros_params_manager_.data().pid_yaw_speed[1],
                  ros_params_manager_.data().pid_yaw_speed[2], ros_params_manager_.data().pid_yaw_speed[3],
                  ros_params_manager_.data().pid_yaw_speed[4]),  // yaw 速度环
              std::make_shared<PosPID>(
                  ros_params_manager_.data().pid_yaw_position[0], ros_params_manager_.data().pid_yaw_position[1],
                  ros_params_manager_.data().pid_yaw_position[2], ros_params_manager_.data().pid_yaw_position[3],
                  ros_params_manager_.data().pid_yaw_position[4]),  // yaw 位置环
              std::make_shared<PosPID>(
                  ros_params_manager_.data().pid_pitch_1_speed[0], ros_params_manager_.data().pid_pitch_1_speed[1],
                  ros_params_manager_.data().pid_pitch_1_speed[2], ros_params_manager_.data().pid_pitch_1_speed[3],
                  ros_params_manager_.data().pid_pitch_1_speed[4]),  // pitch_1 速度环
              std::make_shared<PosPID>(ros_params_manager_.data().pid_pitch_1_position[0],
                                       ros_params_manager_.data().pid_pitch_1_position[1],
                                       ros_params_manager_.data().pid_pitch_1_position[2],
                                       ros_params_manager_.data().pid_pitch_1_position[3],
                                       ros_params_manager_.data().pid_pitch_1_position[4]),  // pitch_1 位置环
              std::make_shared<PosPID>(
                  ros_params_manager_.data().pid_pitch_2_speed[0], ros_params_manager_.data().pid_pitch_2_speed[1],
                  ros_params_manager_.data().pid_pitch_2_speed[2], ros_params_manager_.data().pid_pitch_2_speed[3],
                  ros_params_manager_.data().pid_pitch_2_speed[4]),  // pitch_1 速度环
              std::make_shared<PosPID>(ros_params_manager_.data().pid_pitch_2_position[0],
                                       ros_params_manager_.data().pid_pitch_2_position[1],
                                       ros_params_manager_.data().pid_pitch_2_position[2],
                                       ros_params_manager_.data().pid_pitch_2_position[3],
                                       ros_params_manager_.data().pid_pitch_2_position[4]),  // pitch_1 位置环
              std::make_shared<PosPID>(
                  ros_params_manager_.data().pid_end_yaw_1_speed[0], ros_params_manager_.data().pid_end_yaw_1_speed[1],
                  ros_params_manager_.data().pid_end_yaw_1_speed[2], ros_params_manager_.data().pid_end_yaw_1_speed[3],
                  ros_params_manager_.data().pid_end_yaw_1_speed[4]),  // end_yaw_1 速度环
              std::make_shared<PosPID>(ros_params_manager_.data().pid_end_yaw_1_position[0],
                                       ros_params_manager_.data().pid_end_yaw_1_position[1],
                                       ros_params_manager_.data().pid_end_yaw_1_position[2],
                                       ros_params_manager_.data().pid_end_yaw_1_position[3],
                                       ros_params_manager_.data().pid_end_yaw_1_position[4]),  // end_yaw_1 位置环
              std::make_shared<PosPID>(
                  ros_params_manager_.data().pid_end_pitch_speed[0], ros_params_manager_.data().pid_end_pitch_speed[1],
                  ros_params_manager_.data().pid_end_pitch_speed[2], ros_params_manager_.data().pid_end_pitch_speed[3],
                  ros_params_manager_.data().pid_end_pitch_speed[4]),  // end_pitch 速度环
              std::make_shared<PosPID>(ros_params_manager_.data().pid_end_pitch_position[0],
                                       ros_params_manager_.data().pid_end_pitch_position[1],
                                       ros_params_manager_.data().pid_end_pitch_position[2],
                                       ros_params_manager_.data().pid_end_pitch_position[3],
                                       ros_params_manager_.data().pid_end_pitch_position[4]),  // end_pitch 位置环
              std::make_shared<PosPID>(
                  ros_params_manager_.data().pid_end_yaw_2_speed[0], ros_params_manager_.data().pid_end_yaw_2_speed[1],
                  ros_params_manager_.data().pid_end_yaw_2_speed[2], ros_params_manager_.data().pid_end_yaw_2_speed[3],
                  ros_params_manager_.data().pid_end_yaw_2_speed[4]),  // end_yaw_2 速度环
              std::make_shared<PosRingPID>(ros_params_manager_.data().pid_end_yaw_2_position[0],
                                           ros_params_manager_.data().pid_end_yaw_2_position[1],
                                           ros_params_manager_.data().pid_end_yaw_2_position[2],
                                           ros_params_manager_.data().pid_end_yaw_2_position[3],
                                           ros_params_manager_.data().pid_end_yaw_2_position[4],
                                           M_PI),  // end_yaw_2 位置环
          },
          // joint limits
          {
              std::make_pair(-2.8798, 2.8798),
              std::make_pair(0.0, 2.7911),
              std::make_pair(0.0, 2.7911),
              std::make_pair(-2.6167, 2.6167),
              std::make_pair(-2.09333, 2.09333),
              std::make_pair(-3.14, 3.14),
          }},
      PosPID{static_cast<float>(ros_params_manager_.data().pid_base_yaw_speed[0]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_speed[1]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_speed[2]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_speed[3]),
             static_cast<float>(ros_params_manager_.data().pid_base_yaw_speed[4])},  // base_yaw 速度环
      PosRingPID{static_cast<float>(ros_params_manager_.data().pid_base_yaw_position[0]),
                 static_cast<float>(ros_params_manager_.data().pid_base_yaw_position[1]),
                 static_cast<float>(ros_params_manager_.data().pid_base_yaw_position[2]),
                 static_cast<float>(ros_params_manager_.data().pid_base_yaw_position[3]),
                 static_cast<float>(ros_params_manager_.data().pid_base_yaw_position[4]), M_PI * 2},  // base_yaw 位置环
      std::move(tf_buffer_), this);
  fluorite::actuator = std::make_unique<fluorite::Actuator>(
      Ddm{*arm_can, static_cast<size_t>(ros_params_manager_.data().base_yaw_id)},
      DmMit{*arm_can,
            DmSettingsMit{.master_id = ros_params_manager_.data().yaw_master_id,
                          .slave_id = ros_params_manager_.data().yaw_slave_id,
                          .p_max = 3.141592f,
                          .v_max = 45.f,
                          .t_max = 54.f,
                          .kp_range = {0, 500},
                          .kd_range = {0, 50}},
            ros_params_manager_.data().yaw_reverse},
      DmMit{*arm_can,
            DmSettingsMit{.master_id = ros_params_manager_.data().pitch_1_master_id,
                          .slave_id = ros_params_manager_.data().pitch_1_slave_id,
                          .p_max = 6.283185f,
                          .v_max = 45.f,
                          .t_max = 54.f,
                          .kp_range = {0, 500},
                          .kd_range = {0, 50}},
            ros_params_manager_.data().pitch_1_reverse},
      DmMit{*arm_can,
            DmSettingsMit{.master_id = ros_params_manager_.data().pitch_2_master_id,
                          .slave_id = ros_params_manager_.data().pitch_2_slave_id,
                          .p_max = 3.141592f,
                          .v_max = 45.f,
                          .t_max = 54.f,
                          .kp_range = {0, 500},
                          .kd_range = {0, 50}},
            ros_params_manager_.data().pitch_2_reverse},
      DmMit{*arm_can,
            DmSettingsMit{.master_id = ros_params_manager_.data().end_yaw_1_master_id,
                          .slave_id = ros_params_manager_.data().end_yaw_1_slave_id,
                          .p_max = 3.141592f,
                          .v_max = 30.f,
                          .t_max = 10.f,
                          .kp_range = {0, 500},
                          .kd_range = {0, 50}},
            ros_params_manager_.data().end_yaw_1_reverse},
      DmMit{*arm_can,
            DmSettingsMit{.master_id = ros_params_manager_.data().end_pitch_master_id,
                          .slave_id = ros_params_manager_.data().end_pitch_slave_id,
                          .p_max = 3.141593f,
                          .v_max = 30.f,
                          .t_max = 10.f,
                          .kp_range = {0, 500},
                          .kd_range = {0, 50}},
            ros_params_manager_.data().end_pitch_reverse},
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
  info("objects construction finished \033[32mmok\033[0m");
}

void FluoriteArm::Ins() {
  ins_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
        mahony_.Update(rm::modules::algorithm::ImuData6Dof{
            (float)-msg->angular_velocity.y / 360 * 2 * M_PIf, (float)msg->angular_velocity.z / 360 * 2 * M_PIf,
            (float)msg->angular_velocity.x / 360 * 2 * M_PIf, (float)-msg->linear_acceleration.y,
            (float)msg->linear_acceleration.z, (float)msg->linear_acceleration.x});
        fluorite::state.imu_yaw = mahony_.euler_angle().roll;
        fluorite::state.imu_pitch = mahony_.euler_angle().yaw;
        fluorite::state.imu_roll = mahony_.euler_angle().pitch;
      });
  info("ins \033[32mmok\033[0m");
}

/**
 * @brief 开始接收所有链路的用户输入：sbus、图传链路键鼠、自定义控制器和调试键盘节点（也就是初始化一堆subscriber）
 */
void FluoriteArm::UserInput() {
  sbus_sub_ = create_subscription<sbus_interface::msg::Sbus>(
      "/sbus", rclcpp::SensorDataQoS(), [&](const sbus_interface::msg::Sbus::SharedPtr msg) {
        if (msg->failsafe) {
          fluorite::fsm::DispatchEvent(fluorite::fsm::event::SwitchMode<fluorite::fsm::state::arm::NoForce>{});
          fluorite::fsm::DispatchEvent(fluorite::fsm::event::SwitchMode<fluorite::fsm::state::gimbal_base::NoForce>{});
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
        if (msg->modifiers & keyboard_msgs::msg::Key::MODIFIER_LCTRL) {
          control_msg_.keyboard_key.at(keyboard_msgs::msg::Key::MODIFIER_LCTRL) = true;
        }
        if (msg->modifiers & keyboard_msgs::msg::Key::MODIFIER_LSHIFT) {
          control_msg_.keyboard_key.at(keyboard_msgs::msg::Key::MODIFIER_LSHIFT) = true;
        }
      });
  debug_keyboard_keyup_sub_ = create_subscription<keyboard_msgs::msg::Key>(
      "/keyup", rclcpp::QoS(10).reliable(), [&](const keyboard_msgs::msg::Key::SharedPtr msg) {
        if (control_msg_.keyboard_key.find(msg->code) != control_msg_.keyboard_key.end()) {
          control_msg_.keyboard_key[msg->code] = false;
        }
        if (msg->modifiers & keyboard_msgs::msg::Key::MODIFIER_LCTRL) {
          control_msg_.keyboard_key.at(keyboard_msgs::msg::Key::MODIFIER_LCTRL) = false;
        }
        if (msg->modifiers & keyboard_msgs::msg::Key::MODIFIER_LSHIFT) {
          control_msg_.keyboard_key.at(keyboard_msgs::msg::Key::MODIFIER_LSHIFT) = false;
        }
      });
  fpv_link_input_sub_ = create_subscription<rm2_referee_msgs::msg::RemoteControl>(
      "/rm2_referee/remote_control", rclcpp::SensorDataQoS(), [&](const rm2_referee_msgs::msg::RemoteControl msg) {
        control_msg_.mouse_x = msg.mouse_x / 1000.0;
        control_msg_.mouse_y = msg.mouse_y / 1000.0;
        control_msg_.mouse_wheel = msg.mouse_z / 10.0;
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
  custom_controller_sub_ = create_subscription<rm2_referee_msgs::msg::CustomRobotData>(
      "/rm2_referee/custom_robot_data", rclcpp::SensorDataQoS(), [&](const rm2_referee_msgs::msg::CustomRobotData msg) {
        std::memcpy(&control_msg_.custom_controller_command, &msg.data, sizeof(control_msg_.custom_controller_command));
      });

  info("user input subscribers spawned \033[32mmok\033[0m");
}

void FluoriteArm::Referee() {  // info("referee \033[32mmok\033[0m");
}

/**
 * @brief
 * 发布一些调试信息，比如关节角度、速度、末端目标位置等等。没有实质性作用，不需要debug或者记录log的时候可以把这个timer关掉
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
  end_eff_target_pose_visual_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "fluorite/blackbox/end_effector_target_pose", rclcpp::SystemDefaultsQoS());
  blackbox_timer_ = create_wall_timer(20ms, [this] {
    fluorite::state.header.stamp = now();
    PopulatePidMsg(fluorite::state.pids[0], fluorite::controller->base_yaw_speed_pid);
    PopulatePidMsg(fluorite::state.pids[1], fluorite::controller->base_yaw_position_pid);
    for (size_t i = 0; i < 12; ++i) {
      PopulatePidMsg(fluorite::state.pids[i + 2], *fluorite::controller->trajectory_tracker.pids()[i]);
    }
    blackbox_state_pub_->publish(fluorite::state);

    const auto marker = [&] {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_yaw_link";
      marker.header.stamp = now();
      marker.ns = "end_effector";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = fluorite::state.target_end_effector_pose.position;
      const auto orientation = [&] {
        Eigen::Quaterniond q(fluorite::state.target_end_effector_pose.orientation.w,
                             fluorite::state.target_end_effector_pose.orientation.x,
                             fluorite::state.target_end_effector_pose.orientation.y,
                             fluorite::state.target_end_effector_pose.orientation.z);
        q = q * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
        geometry_msgs::msg::Quaternion o;
        o.x = q.x();
        o.y = q.y();
        o.z = q.z();
        o.w = q.w();
        return o;
      }();
      marker.pose.orientation = orientation;
      marker.scale.x = 0.3;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 1.0;
      if (fluorite::state.last_ik_success) {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      } else {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      }
      return marker;
    }();
    end_eff_target_pose_visual_pub_->publish(marker);
  });
  info("blackbox \033[32mmok\033[0m");
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
  info("device monitor \033[32mmok\033[0m");
}

/**
 * @brief 从电机对象里拿数据，做单位换算、范围处理、滤波器处理等等，最后更新fluorite::state
 */
void FluoriteArm::ProcessSensorData() {
  // 初始化低通滤波器
  static auto lp_filter = [this] {
    struct {
      FO_LPF yaw_position, pitch_1_position, pitch_2_position, end_yaw_1_position, end_pitch_position,
          end_yaw_2_position;
    } ret;
    ret.yaw_position.calculate_coeffs(ros_params_manager_.data().lpf_cutoff_freq_yaw,
                                      ros_params_manager_.data().lpf_sample_rate);
    ret.pitch_1_position.calculate_coeffs(ros_params_manager_.data().lpf_cutoff_freq_pitch_1,
                                          ros_params_manager_.data().lpf_sample_rate);
    ret.pitch_2_position.calculate_coeffs(ros_params_manager_.data().lpf_cutoff_freq_pitch_2,
                                          ros_params_manager_.data().lpf_sample_rate);
    ret.end_yaw_1_position.calculate_coeffs(ros_params_manager_.data().lpf_cutoff_freq_end_yaw_1,
                                            ros_params_manager_.data().lpf_sample_rate);
    ret.end_pitch_position.calculate_coeffs(ros_params_manager_.data().lpf_cutoff_freq_end_pitch,
                                            ros_params_manager_.data().lpf_sample_rate);
    ret.end_yaw_2_position.calculate_coeffs(ros_params_manager_.data().lpf_cutoff_freq_end_yaw_2,
                                            ros_params_manager_.data().lpf_sample_rate);
    return ret;
  }();

  fluorite::state.base_yaw_encoder = rm::modules::algorithm::utils::LoopConstrain(
      fluorite::actuator->base_yaw.pos_rad() - ros_params_manager_.data().base_yaw_encoder_zero_offset, 0.0,
      M_PI * 2.0);
  fluorite::state.base_yaw_encoder_speed = fluorite::actuator->base_yaw.rpm();
  fluorite::state.yaw_encoder = fluorite::actuator->yaw.pos() - ros_params_manager_.data().yaw_encoder_zero_offset;
  fluorite::state.yaw_encoder_speed = fluorite::actuator->yaw.vel();
  fluorite::state.pitch_1_encoder = fluorite::actuator->pitch_1.pos() * (2.0 / 3.0)  // 传动链条 减速比2:3
                                    - ros_params_manager_.data().pitch_1_encoder_zero_offset;
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
  joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("fluorite/arm/joint_states", rclcpp::SystemDefaultsQoS());
  joint_state_msg_.name = {"yaw_joint",       "pitch_1_joint",   "pitch_2_joint",
                           "end_yaw_1_joint", "end_pitch_joint", "end_yaw_2_joint"};
  joint_state_msg_.position.resize(6);
  joint_state_msg_.velocity.resize(6);
  joint_state_pub_timer_ = create_wall_timer(20ms, [this] {
    joint_state_msg_.header.stamp = now();
    joint_state_msg_.position[0] = fluorite::state.yaw_encoder;
    joint_state_msg_.position[1] = fluorite::state.pitch_1_encoder;
    joint_state_msg_.position[2] = fluorite::state.pitch_2_encoder;
    joint_state_msg_.position[3] = fluorite::state.end_yaw_1_encoder;
    joint_state_msg_.position[4] = fluorite::state.end_pitch_encoder;
    joint_state_msg_.position[5] = fluorite::state.end_yaw_2_encoder;
    joint_state_msg_.velocity[0] = fluorite::state.yaw_encoder_speed;
    joint_state_msg_.velocity[1] = fluorite::state.pitch_1_encoder_speed;
    joint_state_msg_.velocity[2] = fluorite::state.pitch_2_encoder_speed;
    joint_state_msg_.velocity[3] = fluorite::state.end_yaw_1_encoder_speed;
    joint_state_msg_.velocity[4] = fluorite::state.end_pitch_encoder_speed;
    joint_state_msg_.velocity[5] = fluorite::state.end_yaw_2_encoder_speed;
    joint_state_pub_->publish(joint_state_msg_);
  });
  info("jsp \033[32mmok\033[0m");
}

/**
 * @brief 发布从imu坐标系(imu)到base_yaw_link坐标系的tf，频率50hz
 */
void FluoriteArm::TFBroadcaster() {
  world_to_base_yaw_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  world_to_base_yaw_tf_.header.frame_id = "imu";
  world_to_base_yaw_tf_.child_frame_id = "base_yaw_link";
  world_to_base_yaw_tf_broadcaster_timer_ = create_wall_timer(20ms, [this] {
    world_to_base_yaw_tf_.header.stamp = now();
    world_to_base_yaw_tf_quat_.setRPY(0, 0, fluorite::state.imu_yaw - fluorite::state.yaw_encoder);
    world_to_base_yaw_tf_.transform.rotation.x = world_to_base_yaw_tf_quat_.x();
    world_to_base_yaw_tf_.transform.rotation.y = world_to_base_yaw_tf_quat_.y();
    world_to_base_yaw_tf_.transform.rotation.z = world_to_base_yaw_tf_quat_.z();
    world_to_base_yaw_tf_.transform.rotation.w = world_to_base_yaw_tf_quat_.w();
    world_to_base_yaw_tf_broadcaster_->sendTransform(world_to_base_yaw_tf_);
  });
  info("tf broadcaster \033[32mmok\033[0m");
}

/**
 * @brief 启动对接行为树节点的action和service服务器，接收行为树节点的控制指令
 */
void FluoriteArm::BehaviourTreeInterface() {
  arm_end_effector_cartesian_command_as_.goal_callback = [this](auto uuid, auto goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  arm_end_effector_cartesian_command_as_.cancel_callback = [this](auto goal_handle) {
    fluorite::fsm::DispatchEvent(fluorite::fsm::event::BehaviorTreeCancelExecution{});
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  arm_end_effector_cartesian_command_as_.accepted_callback = [this](auto goal_handle) {
    fluorite::fsm::DispatchEvent(fluorite::fsm::event::BehaviourTreeCommand{
        *goal_handle->get_goal(),
        [&] {
          if (rclcpp::ok()) {
            auto result = std::make_shared<fluorite_msgs::action::ArmEndEffectorCartesianCommand::Result>();
            result->success = true;
            goal_handle->succeed(result);
          }
        },
        [&] {
          auto result = std::make_shared<fluorite_msgs::action::ArmEndEffectorCartesianCommand::Result>();
          result->success = false;
          goal_handle->canceled(result);
        }});
  };
  arm_joint_command_as_.goal_callback = [this](auto uuid, auto goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  arm_joint_command_as_.cancel_callback = [this](auto goal_handle) {
    fluorite::fsm::DispatchEvent(fluorite::fsm::event::BehaviorTreeCancelExecution{});
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  arm_joint_command_as_.accepted_callback = [this](auto goal_handle) {
    fluorite::fsm::DispatchEvent(fluorite::fsm::event::BehaviourTreeCommand{
        *goal_handle->get_goal(),
        [&] {
          if (rclcpp::ok()) {
            auto result = std::make_shared<fluorite_msgs::action::ArmJointCommand::Result>();
            result->success = true;
            goal_handle->succeed(result);
          }
        },
        [&] {
          auto result = std::make_shared<fluorite_msgs::action::ArmJointCommand::Result>();
          result->success = false;
          goal_handle->canceled(result);
        }});
  };
  arm_trajectory_command_as_.goal_callback = [this](auto uuid, auto goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  arm_trajectory_command_as_.cancel_callback = [this](auto goal_handle) {
    fluorite::fsm::DispatchEvent(fluorite::fsm::event::BehaviorTreeCancelExecution{});
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  arm_trajectory_command_as_.accepted_callback = [this](auto goal_handle) {
    fluorite::fsm::DispatchEvent(fluorite::fsm::event::BehaviourTreeCommand{
        *goal_handle->get_goal(),
        [&] {
          if (rclcpp::ok()) {
            auto result = std::make_shared<fluorite_msgs::action::ArmTrajectoryCommand::Result>();
            result->success = true;
            goal_handle->succeed(result);
          }
        },
        [&] {
          auto result = std::make_shared<fluorite_msgs::action::ArmTrajectoryCommand::Result>();
          result->success = false;
          goal_handle->canceled(result);
        }});
  };

  arm_end_effector_cartesian_command_as_.Create(this, "fluorite/arm/end_effector_cartesian_command");
  arm_joint_command_as_.Create(this, "fluorite/arm/joint_command");
  arm_trajectory_command_as_.Create(this, "fluorite/arm/trajectory_command");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteArm>());
  rclcpp::shutdown();
  return 0;
}
