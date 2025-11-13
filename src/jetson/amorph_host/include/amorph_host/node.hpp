
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ros_params_manager.hpp"
#include "rpc/rpc.hpp"
#include "logger.hpp"
#include "amorph_host/ros_params.hpp"
#include "amorph_host/ros_params_manager.hpp"
#include "mtc/pick_place_task/pick_place_task.hpp"

class AmorphArmNode {
 public:
  AmorphArmNode()
      : ros_node_{std::make_shared<rclcpp::Node>("amorph_arm_node",                      //
                                                 rclcpp::NodeOptions()                   //
                                                     .allow_undeclared_parameters(true)  //
                                                     .automatically_declare_parameters_from_overrides(true))} {
    params_manager_ = std::make_unique<RosParamsManager<RosParams>>(ros_node_);
    params_manager_->Init();
    ros_params_ = params_manager_->data();
    logger_.Init(ros_node_);
    logger_.Info("'%s' has been created.", ros_node_->get_name());
    // InitRpc();
    moveit_task_constructor_demo::PickPlaceTask t(ros_node_, "pick_place_task");
    t.Init(ros_node_);
    if (t.Plan(10)) {
      t.task()->publishAllSolutions(true);
    }
  }

  ~AmorphArmNode() { logger_.Info("'%s' is being destroyed.", ros_node_->get_name()); }
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

  std::unique_ptr<RosParamsManager<RosParams>> params_manager_;
  RosParams ros_params_;
  rclcpp::Node::SharedPtr ros_node_{nullptr};
  RosLogger& logger_{RosLogger::GetInstance()};
  rm::hal::Can rpc_can_{"can0"};
  RPC rpc_;
};