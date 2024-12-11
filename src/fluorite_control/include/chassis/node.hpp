
#ifndef CHASSIS_NODE_HPP
#define CHASSIS_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rm2_referee_msgs/msg/robot_status.hpp>
#include <rm2_referee_msgs/srv/tx.hpp>
#include <librm.hpp>

#include "chassis/fsm.hpp"
#include "chassis/ros_params.hpp"
#include "common/device_monitor.hpp"
#include "common/ros_params_manager.hpp"

/**
 * @brief    控制节点
 */
class FluoriteChassis final : public rclcpp::Node {
 public:
  FluoriteChassis(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
                                                           .allow_undeclared_parameters(true)
                                                           .automatically_declare_parameters_from_overrides(true));
  ~FluoriteChassis() override;

 private:
  void InitParameters();
  void InitFsm();

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
  RosParamsManager<RosParams> ros_params_manager_{this};
  /** </parameters> **/

  /** <remote> **/
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr remote_sub_{nullptr};
  /** </remote> **/

  /** <referee> **/
  rclcpp::Subscription<rm2_referee_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_{nullptr};
  /** </referee> **/

  /** <supercap> **/
  // std::unique_ptr<rm::device::SuperCap> supercap_{nullptr};
  rclcpp::TimerBase::SharedPtr supercap_timer_{nullptr};
  /** </supercap> **/

  /** <nav comm> **/
  rclcpp::TimerBase::SharedPtr nav_comm_timer_{nullptr};
  // TODO
  /** </vision comm> **/

  /** <blackbox> **/
  rclcpp::TimerBase::SharedPtr blackbox_timer_{nullptr};
  rclcpp::Publisher<fluorite_msgs::msg::ChassisState>::SharedPtr blackbox_state_pub_{nullptr};
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
  event::chassis::Control control_msg_;
  /** </control loop> **/
};

#endif