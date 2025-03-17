
#ifndef ARM_NODE_HPP
#define ARM_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sbus_interface/msg/sbus.hpp>
#include <rm2_referee_msgs/msg/robot_status.hpp>
#include <rm2_referee_msgs/srv/tx.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <librm.hpp>

#include "arm/fsm.hpp"
#include "arm/ros_params.hpp"
#include "common/device_monitor.hpp"
#include "common/ros_params_manager.hpp"
#include "librm/modules/algorithm/ahrs/mahony.h"

/**
 * @brief    控制节点
 */
class FluoriteArm final : public rclcpp::Node {
 public:
  FluoriteArm(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
                                                       .allow_undeclared_parameters(true)
                                                       .automatically_declare_parameters_from_overrides(true));
  ~FluoriteArm() override;

 private:
  void InitParameters();
  void InitializeShit();

  void Blackbox();
  void DeviceMonitor();
  void Ins();
  void Remote();
  void Keyboard();
  void Referee();
  void JointStatesPublisher();
  void TFBroadcaster();

  void ParamHotUpdate();
  void ProcessSensorData();

  // auto ParameterCallback(const std::vector<rclcpp::Parameter> &parameters) ->
  // rcl_interfaces::msg::SetParametersResult;

 private:
  /** <parameters> **/
  RosParamsManager<RosParams> ros_params_manager_{reinterpret_cast<rclcpp::Node *>(this)};
  /** </parameters> **/

  /** <tf publishers> **/
  std::unique_ptr<tf2_ros::TransformBroadcaster> world_to_base_yaw_tf_broadcaster_{nullptr};
  rclcpp::TimerBase::SharedPtr world_to_base_yaw_tf_broadcaster_timer_{nullptr};
  tf2::Quaternion world_to_base_yaw_tf_quat_{};
  geometry_msgs::msg::TransformStamped world_to_base_yaw_tf_{};
  /** </tf publishers> **/

  /** <ins> **/
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ins_sub_{nullptr};
  rm::modules::algorithm::MahonyAhrs mahony_{};
  /** </ins> **/

  /** <user input subscribers> **/
  rclcpp::Subscription<sbus_interface::msg::Sbus>::SharedPtr remote_sub_{nullptr};
  rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keyboard_keydown_sub_{nullptr};
  rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keyboard_keyup_sub_{nullptr};
  /** </user input subscribers> **/

  /** <referee> **/
  rclcpp::Subscription<rm2_referee_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_{nullptr};
  /** </referee> **/

  /** <blackbox> **/
  rclcpp::TimerBase::SharedPtr blackbox_timer_{nullptr};
  rclcpp::Publisher<fluorite_msgs::msg::ArmState>::SharedPtr blackbox_state_pub_{nullptr};
  /** </blackbox> **/

  /** <device monitor> **/
  class DeviceMonitor device_monitor_;
  Device remote_device_{"remote"};
  rclcpp::TimerBase::SharedPtr device_monitor_timer_{nullptr};
  /** </device monitor> **/

  /** <referee tx> **/
  rm2_referee_msgs::srv::Tx::Request referee_tx_data_{};
  rclcpp::Client<rm2_referee_msgs::srv::Tx>::SharedPtr referee_client_{nullptr};
  /** </referee tx> **/

  /** <control loop> **/
  rclcpp::TimerBase::SharedPtr control_loop_timer_{nullptr};
  fluorite::fsm::event::Control control_msg_;
  /** </control loop> **/

  /** <joint states publisher> **/
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_{nullptr};
  rclcpp::TimerBase::SharedPtr joint_states_pub_timer_{nullptr};
  sensor_msgs::msg::JointState joint_states_msg_{};
  /** </joint states publisher> **/
};

#endif