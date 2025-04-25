
#ifndef NODES_ACTION_PLAN_TO_JOINT_POSITION_HPP
#define NODES_ACTION_PLAN_TO_JOINT_POSITION_HPP

#include <utility>
#include <thread>
#include <future>

#include "behaviortree_cpp_v3/action_node.h"

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "moveit.hpp"

/**
 * @brief   Action Node: 尝试从当前位置规划一条轨迹到指定的关节位置
 * @details Input: joint_positions
 *          Output: trajectory
 */
class ActionPlanToJointPosition : public BT::StatefulActionNode {
 public:
  static constexpr const char* name = "ActionPlanToJointPosition";

  ActionPlanToJointPosition(const std::string& name, const BT::NodeConfiguration& config)
      : StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<double>>("joint_positions"),
        BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("trajectory"),
    };
  }

  BT::NodeStatus onStart() override {
    BT::Optional<std::vector<double>> msg = getInput<std::vector<double>>("joint_positions");
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }
    if (msg.value().size() != 6) {
      throw BT::RuntimeError("joint_positions size is not 6");
    }
    std::cout << JsonBuilder::BuildJson(msg.value()) << std::endl;

    planning_future_ = std::async(std::launch::async, [&] {
      auto& mgi = MoveGroupInterfaceSingleton::GetInstance().GetMoveGroupInterface();
      try {
        mgi.setJointValueTarget(msg.value());
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto kOk = mgi.plan(plan);
        MoveGroupInterfaceSingleton::GetInstance().ReleaseMoveGroupInterface();
        return std::make_pair(kOk, plan);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("fluorite_behaviour_tree"), "Error: %s", e.what());
        MoveGroupInterfaceSingleton::GetInstance().ReleaseMoveGroupInterface();
        return std::make_pair(moveit::core::MoveItErrorCode{moveit::core::MoveItErrorCode::FAILURE},
                              moveit::planning_interface::MoveGroupInterface::Plan());
      }
    });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (planning_future_.valid()) {  // future有效说明决策树现在正在等待规划结果
      if (planning_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
          const auto [ok, plan] = planning_future_.get();
          if (ok != moveit::core::MoveItErrorCode::SUCCESS) {
            return BT::NodeStatus::FAILURE;
          }
          setOutput("trajectory", plan.trajectory_.joint_trajectory);
          return BT::NodeStatus::SUCCESS;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(rclcpp::get_logger("fluorite"), "Error: %s", e.what());
          return BT::NodeStatus::FAILURE;
        }
      } else {
        return BT::NodeStatus::RUNNING;
      }
    } else {
      throw BT::RuntimeError("future is not valid");
    }
  }

  void onHalted() override {}

 private:
  using PlanningResult = std::pair<moveit::core::MoveItErrorCode, moveit::planning_interface::MoveGroupInterface::Plan>;
  std::future<PlanningResult> planning_future_;
};

#endif