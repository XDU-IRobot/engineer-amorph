
#ifndef ARM_NODE_HPP
#define ARM_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sbus_interface/msg/sbus.hpp>
#include <rm2_referee_msgs/msg/robot_status.hpp>
#include <rm2_referee_msgs/srv/tx.hpp>
#include <librm.hpp>

#include "arm/fsm.hpp"
#include "arm/ros_params.hpp"
#include "common/device_monitor.hpp"
#include "common/ros_params_manager.hpp"

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

  void Navigation();
  void Blackbox();
  void DeviceMonitor();
  void Remote();
  void Referee();

  void ParamHotUpdate();
  void UpdateFsmState();

  auto ParameterCallback(const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult;

 private:
  /** <parameters> **/
  RosParamsManager<RosParams> ros_params_manager_{reinterpret_cast<rclcpp::Node *>(this)};
  /** </parameters> **/

  /** <remote> **/
  rclcpp::Subscription<sbus_interface::msg::Sbus>::SharedPtr remote_sub_{nullptr};
  /** </remote> **/

  /** <referee> **/
  rclcpp::Subscription<rm2_referee_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_{nullptr};
  /** </referee> **/

  /** <nav comm> **/
  rclcpp::TimerBase::SharedPtr nav_comm_timer_{nullptr};
  // TODO
  /** </vision comm> **/

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
};

#endif