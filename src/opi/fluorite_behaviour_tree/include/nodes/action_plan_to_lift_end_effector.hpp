#ifndef NODES_ACTION_PLAN_TO_LIFT_END_EFFECTOR_HPP
#define NODES_ACTION_PLAN_TO_LIFT_END_EFFECTOR_HPP

#include <utility>
#include <thread>
#include <future>

#include "behaviortree_cpp_v3/action_node.h"

#include "tweeny.h"

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "moveit.hpp"
#include "ros_node.hpp"
#include "easing.hpp"

/**
 * @brief   Action Node: 尝试规划一条让末端执行器沿直线升起一定距离（单位m）的轨迹，吸到银矿之后抬手的时候用
 * @details Input: lift_distance
 *          Output: trajectory
 */
class ActionPlanToLiftEndEffector : public BT::StatefulActionNode {
 public:
  static constexpr const char* name = "ActionPlanToLiftEndEffector";

  ActionPlanToLiftEndEffector(const std::string& name, const BT::NodeConfiguration& config)
      : StatefulActionNode(name, config), ros_logger_{RosNodeSingleton::GetInstance().GetNode()->get_logger()} {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("max_linear_speed"),
        BT::InputPort<double>("lift_distance"),
        BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("trajectory"),
    };
  }

  BT::NodeStatus onStart() override {
    BT::Optional<double> lift_distance = getInput<double>("lift_distance");
    if (!lift_distance) {
      throw BT::RuntimeError("missing required input [lift_distance]: ", lift_distance.error());
    }
    BT::Optional<double> max_linear_speed = getInput<double>("max_linear_speed");
    if (!max_linear_speed) {
      throw BT::RuntimeError("missing required input [max_linear_speed]: ", max_linear_speed.error());
    }

    planning_future_ = std::async(
        // lift_distance_value和max_linear_speed需要被传进来！lambda中取值时，onStart函数肯定已经返回了，直接捕获的话会访问到已经被销毁的这两个变量
        [this](double lift_distance_value, double max_linear_speed_value) {
          auto& mgi = MoveitHelper::GetInstance().GetMoveGroupInterface();
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
            RCLCPP_INFO(ros_logger_, "lift: %f, current z: %f, lifted z: %f", lift_distance_value,
                        kCurrentPose.position.z, kPoseLifted.position.z);
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
                mgi.computeCartesianPath(kWaypoints, kEEFStep, kJumpThreshold, trajectory, false, &error_code);
            if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
              ok = moveit::core::MoveItErrorCode::SUCCESS;

              // 如果轨迹里只有一个点那要么是plan失败了，要么是真的起点和终点离得很近
              // 按使用场景来看起点和终点一定不会太近，所以如果轨迹只有一个点一律视为规划失败
              if (trajectory.joint_trajectory.points.size() <= 1) {
                MoveitHelper::GetInstance().ReleaseMoveGroupInterface();
                return std::make_pair(moveit::core::MoveItErrorCode{moveit::core::MoveItErrorCode::FAILURE},
                                      trajectory_msgs::msg::JointTrajectory());
              }
              /// 规划Cartesian路径成功，接下来给每个点赋上时间戳

              // 为了计算简单，取max_linear_speed/2为平均速度
              const double kAverageSpeed = max_linear_speed_value / 2.0;
              // 所以轨迹的总时间就是点数*点间距/平均速度
              const size_t kPointCount = trajectory.joint_trajectory.points.size();
              const auto kDuration = kPointCount * kEEFStep / kAverageSpeed;
              auto timestamp_tween = tweeny::from(0.0)
                                         .to(kDuration)
                                         .during(kPointCount - 1)
                                         .via(SineInOutEasingInversed<double>);  // https://easings.net/zh-cn
              RCLCPP_INFO(ros_logger_, "waypoints: %zu, trajectory duration: %.2fs @max eef linear speed: %.2fm/s",
                          kPointCount, kDuration, max_linear_speed_value);
              for (int i = 0; i < kPointCount; ++i) {
                auto& point = trajectory.joint_trajectory.points[i];
                const auto kTimeStamp = timestamp_tween.seek(i);
                point.time_from_start = rclcpp::Duration::from_seconds(kTimeStamp);
              }

              MoveitHelper::GetInstance().DrawTrajectory(trajectory.joint_trajectory);
            } else {
              ok = moveit::core::MoveItErrorCode::FAILURE;
            }
            MoveitHelper::GetInstance().ReleaseMoveGroupInterface();
            return std::make_pair(ok, trajectory.joint_trajectory);
          } catch (const std::exception& e) {
            RCLCPP_ERROR(ros_logger_, "Error: %s", e.what());
            MoveitHelper::GetInstance().ReleaseMoveGroupInterface();
            return std::make_pair(moveit::core::MoveItErrorCode{moveit::core::MoveItErrorCode::FAILURE},
                                  trajectory_msgs::msg::JointTrajectory());
          }
        },
        lift_distance.value(), max_linear_speed.value());
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
          RCLCPP_ERROR(ros_logger_, "Error: %s", e.what());
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
  rclcpp::Logger ros_logger_;
};

#endif
