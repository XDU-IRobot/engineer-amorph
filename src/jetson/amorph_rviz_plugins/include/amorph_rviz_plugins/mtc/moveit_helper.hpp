#pragma once

#include <chrono>
#include <utility>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "logger.hpp"

namespace amorph {

/**
 * @brief 简化 MoveIt 操作的辅助类（单例）
 */
class MoveitHelper {
 public:
  MoveitHelper(const MoveitHelper &) = delete;
  MoveitHelper &operator=(const MoveitHelper &) = delete;

  static MoveitHelper &GetInstance() {
    static MoveitHelper instance;
    return instance;
  }

  void Init(const rclcpp::Node::SharedPtr &node) {
    if (node_) {
      L::W("MoveitHelper already initialized, skipping");
      return;
    }
    node_ = node;
    L::I("MoveitHelper initialized");
  }

  auto CalculateIK(const geometry_msgs::msg::Pose &eff_pose,
                   const std::string &joint_model_group_name,
                   std::chrono::duration<double> timeout = std::chrono::milliseconds(100)) const
      -> std::pair<bool, trajectory_msgs::msg::JointTrajectoryPoint> {
    try {
      robot_model_loader::RobotModelLoader robot_model_loader{node_};
      const auto &kinematic_model = robot_model_loader.getModel();
      const auto robot_state = std::make_shared<moveit::core::RobotState>(kinematic_model);

      const auto *const joint_model_group = robot_state->getJointModelGroup(joint_model_group_name);
      const bool ik_solved = robot_state->setFromIK(
          joint_model_group, eff_pose,
          std::chrono::duration_cast<std::chrono::seconds>(timeout).count());
      robot_state->enforceBounds();
      if (ik_solved && robot_state->satisfiesBounds()) {
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(joint_model_group, joint_values);
        return {true, [&] {
          trajectory_msgs::msg::JointTrajectoryPoint p;
          p.positions = joint_values;
          p.velocities.resize(6);
          return p;
        }()};
      }
      L::W("failed to solve ik");
    } catch (const std::exception &e) {
      L::E("exception during ik calculation: %s", e.what());
    }
    return {false, trajectory_msgs::msg::JointTrajectoryPoint{}};
  }

  void SpawnObject(const moveit_msgs::msg::CollisionObject &object) {
    if (!psi_.applyCollisionObject(object)) {
      throw std::runtime_error("Failed to spawn object: " + object.id);
    }
  }

  void RemoveObject(const std::string &object_id) {
    psi_.removeCollisionObjects({object_id});
  }

  void ClearPlanningScene() {
    auto known_objects = psi_.getKnownObjectNames();
    if (!known_objects.empty()) {
      psi_.removeCollisionObjects(known_objects);
    }
  }

  static Eigen::Isometry3d VectorToEigen(const std::vector<double> &values) {
    if (values.size() != 6) {
      throw std::runtime_error("VectorToEigen requires a vector of size 6");
    }
    return Eigen::Translation3d(values[0], values[1], values[2]) *
           Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
  }

  static geometry_msgs::msg::Pose VectorToPose(const std::vector<double> &values) {
    return tf2::toMsg(VectorToEigen(values));
  }

  moveit::planning_interface::PlanningSceneInterface &psi() { return psi_; }

 private:
  MoveitHelper() = default;

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::PlanningSceneInterface psi_;
};

}  // namespace amorph
