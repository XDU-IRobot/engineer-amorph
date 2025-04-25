
#ifndef NODES_ACTION_PLAN_TO_END_EFFECTOR_POSE_HPP
#define NODES_ACTION_PLAN_TO_END_EFFECTOR_POSE_HPP

#include <utility>
#include <thread>
#include <future>

#include "behaviortree_cpp_v3/action_node.h"

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "moveit.hpp"

/**
 * @brief   Action Node: 尝试从当前位置规划一条轨迹到指定的末端执行器位姿
 * @details Input: pose
 *          Output: trajectory
 */
class ActionPlanToEndEffectorPose : public BT::StatefulActionNode {
 public:
  static constexpr const char* name = "ActionPlanToEndEffectorPose";

  ActionPlanToEndEffectorPose(const std::string& name, const BT::NodeConfiguration& config)
      : StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("srdf_pose_name"),
        BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("trajectory"),
    };
  }

  BT::NodeStatus onStart() override {
    BT::Optional<std::string> msg = getInput<std::string>("srdf_pose_name");
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }

    planning_future_ = std::async(std::launch::async, [&] {
      auto& mgi = MoveGroupInterfaceSingleton::GetInstance().GetMoveGroupInterface();
      auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.28;
        msg.position.y = -0.2;
        msg.position.z = 0.5;
        return msg;
      }();
      mgi.setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto ok = mgi.plan(plan);
      return std::make_pair(ok, plan);
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