
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "rpc/rpc.hpp"
#include "logger.hpp"
#include "amorph_host/amorph_host_parameters.hpp"

class ArmNode {
 public:
  ArmNode() {
    ros_node_ = std::make_shared<rclcpp::Node>("amorph_arm_node",                      //
                                               rclcpp::NodeOptions()                   //
                                                   .allow_undeclared_parameters(true)  //
                                                   .automatically_declare_parameters_from_overrides(true));
    logger_.Init(ros_node_);
    logger_.Info("ArmNode '%s' has been created.", ros_node_->get_name());
    ros_param_listener_ = std::make_unique<amorph_host::ParamListener>(ros_node_->shared_from_this());
    ros_params_ = ros_param_listener_->get_params();
    InitRpc();
  }

  ~ArmNode() { logger_.Info("ArmNode '%s' is being destroyed.", ros_node_->get_name()); }
  auto ros_node() { return ros_node_; }

 private:
  void InitRpc() {
    auto fdcan_transport = std::make_shared<FdCanTransport>(rpc_can_,                        //
                                                            ros_params_.rpc.miso_frame_id,   //
                                                            ros_params_.rpc.mosi_frame_id);  //
    rpc_.AttachToConnection(fdcan_transport);
    rpc_can_.Begin();

    rpc_.rpc()->subscribe("log_message", [this](const LogMessage& msg) -> void {
      switch (msg.log_level) {
        case LogMessage::LogLevel::kDebug:
          logger_.Debug("[stm32] %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kInfo:
          logger_.Info("[stm32] %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kWarn:
          logger_.Warn("[stm32] %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kError:
          logger_.Error("[stm32] %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kFatal:
          logger_.Fatal("[stm32] %s", msg.str.c_str());
          break;
        default:
          logger_.Info("[stm32] %s", msg.str.c_str());
          break;
      }
    });
  }

  rclcpp::Node::SharedPtr ros_node_{nullptr};
  std::unique_ptr<amorph_host::ParamListener> ros_param_listener_;
  amorph_host::Params ros_params_;
  RosLogger& logger_{RosLogger::GetInstance()};
  rm::hal::Can rpc_can_{"can0"};
  RPC rpc_;
};