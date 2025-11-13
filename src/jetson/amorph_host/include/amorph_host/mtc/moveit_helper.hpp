
#pragma once

#include <chrono>
#include <utility>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

inline void SpawnObject(moveit::planning_interface::PlanningSceneInterface &psi,
                        const moveit_msgs::msg::CollisionObject &object) {
  if (!psi.applyCollisionObject(object)) {
    throw std::runtime_error("Failed to spawn object: " + object.id);
  }
}

inline Eigen::Isometry3d VectorToEigen(const std::vector<double> &values) {
  if (values.size() != 6) {
    throw std::runtime_error("VectorToEigen requires a vector of size 6");
  }
  return Eigen::Translation3d(values[0], values[1], values[2]) *
         Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}

inline geometry_msgs::msg::Pose VectorToPose(const std::vector<double> &values) {
  return tf2::toMsg(VectorToEigen(values));
};

class MoveitHelper {
 public:
  explicit MoveitHelper(rclcpp::Node *node, std::string joint_model_group = "fluorite_arm")
      : node_{node}, node_logger_{node->get_logger()}, joint_model_group_{std::move(joint_model_group)} {
    robot_model_loader::RobotModelLoader robot_model_loader{node_};
    kinematic_model_ = robot_model_loader.getModel();
    robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
  }

  auto CalculateIK(const geometry_msgs::msg::Pose &eff_pose,
                   std::chrono::duration<double> timeout = std::chrono::milliseconds(100)) const
      -> std::pair<bool,                                        // success?
                   trajectory_msgs::msg::JointTrajectoryPoint>  // result
  {
    try {
      // 尝试逆解，成功的话就把解出来的关节位置送进轨迹跟踪器
      const auto *const joint_model_group = robot_state_->getJointModelGroup(joint_model_group_);
      const bool ik_solved = robot_state_->setFromIK(joint_model_group, eff_pose,
                                                     std::chrono::duration_cast<std::chrono::seconds>(timeout).count());
      robot_state_->enforceBounds();
      if (ik_solved && robot_state_->satisfiesBounds()) {
        std::vector<double> joint_values;
        robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
        RCLCPP_DEBUG(node_logger_, "====================================================");
        RCLCPP_DEBUG(node_logger_, "ik solved!");
        for (std::size_t i = 0; i < joint_values.size(); ++i) {
          RCLCPP_DEBUG(node_logger_, "Joint %lu: %f", i, joint_values[i]);
        }
        RCLCPP_DEBUG(node_logger_, "====================================================");
        const auto result = [&] {
          trajectory_msgs::msg::JointTrajectoryPoint p;
          p.positions = joint_values;
          p.velocities.resize(6);
          return p;
        }();
        return std::make_pair(true, result);
      }
      RCLCPP_WARN(node_logger_, "failed to solve ik");  // 无解

    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_logger_, "exception occured during ik calculation: %s", e.what());  // 求解过程中有报错
    }
    return std::make_pair(false, trajectory_msgs::msg::JointTrajectoryPoint{});
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger node_logger_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr robot_state_;
  std::string joint_model_group_;
};
