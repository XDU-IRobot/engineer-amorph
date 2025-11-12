
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "rpc/rpc.hpp"
#include "common/ros_params_manager.hpp"
#include "common/logger.hpp"
#include "ros_params.hpp"

class ArmNode : public rclcpp::Node {
 public:
  explicit ArmNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
                                                            .allow_undeclared_parameters(true)
                                                            .automatically_declare_parameters_from_overrides(true))
      : Node("amorph_arm_node", options) {
    amorph::info("ArmNode '%s' has been created.", this->get_name());

    params_manager_.Init();
    InitRpc();
  }

  ~ArmNode() override { amorph::info("ArmNode '%s' is being destroyed.", this->get_name()); }

 private:
  void InitRpc() {
    auto fdcan_transport = std::make_shared<FdCanTransport>(rpc_can_, 0x23, 0x33);
    rpc_.AttachToConnection(fdcan_transport);
    rpc_can_.Begin();

    rpc_.rpc()->subscribe("log_message", [](const LogMessage& msg) -> void {
      switch (msg.log_level) {
        case LogMessage::LogLevel::kDebug:
          amorph::debug("[stm32] %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kInfo:
          amorph::info("[stm32] %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kWarn:
          amorph::warn("[stm32] %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kError:
          amorph::error("[stm32] %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kFatal:
          amorph::fatal("[stm32] %s", msg.str.c_str());
          break;
        default:
          amorph::info("[stm32] %s", msg.str.c_str());
          break;
      }
    });
  }

  RosParamsManager<RosParams> params_manager_{this};
  rm::hal::Can rpc_can_{"can0"};
  RPC rpc_;
};