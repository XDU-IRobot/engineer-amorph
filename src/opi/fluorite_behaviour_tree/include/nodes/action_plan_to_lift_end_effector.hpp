
#ifndef NODES_ACTION_PLAN_TO_LIFT_END_EFFECTOR_HPP
#define NODES_ACTION_PLAN_TO_LIFT_END_EFFECTOR_HPP

#include <utility>
#include <thread>
#include <future>

#include "behaviortree_cpp_v3/action_node.h"

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "moveit.hpp"
#include "ros_node.hpp"

/**
 * @brief   Action Node: 尝试规划一条让末端执行器沿直线升起一定距离（单位m）的轨迹，吸到银矿之后抬手的时候用
 * @details Input: lift_distance
 *          Output: trajectory
 */
class ActionPlanToLiftEndEffector : public BT::StatefulActionNode {
 public:
  static constexpr const char* name = "ActionPlanToLiftEndEffector";

  ActionPlanToLiftEndEffector(const std::string& name, const BT::NodeConfiguration& config)
      : StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("lift_distance"),
        BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("trajectory"),
    };
  }

  BT::NodeStatus onStart() override {
    BT::Optional<double> lift_distance = getInput<double>("lift_distance");
    if (!lift_distance) {
      throw BT::RuntimeError("missing required input [lift_distance]: ", lift_distance.error());
    }

    planning_future_ = std::async(
        // lift_distance_value需要被传进来！lambda中取lift_distance_value的值时，onStart函数肯定已经返回了，直接捕获的话会访问到已经被销毁的lift_distance_value
        [this](double lift_distance_value) {
          auto& mgi = MoveGroupInterfaceSingleton::GetInstance().GetMoveGroupInterface();
          try {
            const auto kCurrentPose = [&] {
              const auto kPoseStamped = mgi.getCurrentPose();
              geometry_msgs::msg::Pose ret;
              ret.position.x = kPoseStamped.pose.position.x;
              ret.position.y = kPoseStamped.pose.position.y;
              ret.position.z = kPoseStamped.pose.position.z;
              ret.orientation.x = kPoseStamped.pose.orientation.x;
              ret.orientation.y = kPoseStamped.pose.orientation.y;
              ret.orientation.z = kPoseStamped.pose.orientation.z;
              ret.orientation.w = kPoseStamped.pose.orientation.w;
              return ret;
            }();
            const auto kPoseLifted = [&] {
              geometry_msgs::msg::Pose pose_lifted = kCurrentPose;
              pose_lifted.position.z += lift_distance_value;
              return pose_lifted;
            }();
            RosNodeSingleton::Info("lift: %f", lift_distance_value);
            RosNodeSingleton::Info("current z: %f", kPoseLifted.position.z);
            RosNodeSingleton::Info("lifted z: %f", kPoseLifted.position.z);
            const std::vector<geometry_msgs::msg::Pose> kWaypoints{
                kCurrentPose,
                kPoseLifted,
            };
            moveit_msgs::msg::RobotTrajectory trajectory;
            moveit_msgs::msg::MoveItErrorCodes error_code;
            moveit::core::MoveItErrorCode ok;
            const double kJumpThreshold = 0.0;
            const double kEEFStep = 0.01;
            double fraction =
                mgi.computeCartesianPath(kWaypoints, kEEFStep, kJumpThreshold, trajectory, true, &error_code);
            if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
              ok = moveit::core::MoveItErrorCode::SUCCESS;
            } else {
              ok = moveit::core::MoveItErrorCode::FAILURE;
            }
            MoveGroupInterfaceSingleton::GetInstance().ReleaseMoveGroupInterface();
            return std::make_pair(ok, trajectory.joint_trajectory);
          } catch (const std::exception& e) {
            RosNodeSingleton::Error("Error: %s", e.what());
            MoveGroupInterfaceSingleton::GetInstance().ReleaseMoveGroupInterface();
            return std::make_pair(moveit::core::MoveItErrorCode{moveit::core::MoveItErrorCode::FAILURE},
                                  trajectory_msgs::msg::JointTrajectory());
          }
        },
        lift_distance.value());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (planning_future_.valid()) {
      if (planning_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
          const auto [ok, trajectory] = planning_future_.get();
          if (ok != moveit::core::MoveItErrorCode::SUCCESS) {
            return BT::NodeStatus::FAILURE;
          }
          setOutput("trajectory", trajectory);
          return BT::NodeStatus::SUCCESS;
        } catch (const std::exception& e) {
          RosNodeSingleton::Error("Error: %s", e.what());
          return BT::NodeStatus::FAILURE;
        }
      } else {
        return BT::NodeStatus::RUNNING;
      }
    } else {
      throw BT::RuntimeError("future is not valid, this should never happen!");
    }
  }

  void onHalted() override {}

 private:
  using PlanningResult = std::pair<moveit::core::MoveItErrorCode, trajectory_msgs::msg::JointTrajectory>;
  std::future<PlanningResult> planning_future_;
};

#endif