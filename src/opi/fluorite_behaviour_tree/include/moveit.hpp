/**
 * @file  moveit.hpp
 * @brief 本文件定义轨迹、点等等数据结构的json序列化和反序列化器，并且提供一个Moveit相关对象的单例供决策树节点使用
 */

#ifndef MOVEIT_HPP
#define MOVEIT_HPP

#include <memory>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <behaviortree_cpp_v3/basic_types.h>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

class JsonBuilder {
 public:
  JsonBuilder() = delete;
  JsonBuilder(const JsonBuilder &) = delete;
  JsonBuilder(JsonBuilder &&) = delete;
  JsonBuilder &operator=(const JsonBuilder &) = delete;

  template <typename T>
  static rapidjson::Value BuildJsonArray(const std::vector<T> &vec, rapidjson::Document::AllocatorType &allocator) {
    rapidjson::Value arr(rapidjson::kArrayType);
    for (const auto &val : vec) {
      arr.PushBack(val, allocator);
    }
    return arr;
  }

  static rapidjson::Value BuildJsonArray(const std::vector<double> &vec,
                                         rapidjson::Document::AllocatorType &allocator) {
    rapidjson::Value arr(rapidjson::kArrayType);
    for (const auto &val : vec) {
      arr.PushBack(val, allocator);
    }
    return arr;
  }

  static std::string BuildJson(const trajectory_msgs::msg::JointTrajectory &trajectory) {
    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();

    rapidjson::Value points(rapidjson::kArrayType);
    for (const auto &point : trajectory.points) {
      rapidjson::Value point_obj(rapidjson::kObjectType);
      point_obj.AddMember("time_from_start", point.time_from_start.sec + point.time_from_start.nanosec / 1e9,
                          allocator);

      point_obj.AddMember("positions", BuildJsonArray(point.positions, allocator), allocator);
      point_obj.AddMember("velocities", BuildJsonArray(point.velocities, allocator), allocator);
      point_obj.AddMember("accelerations", BuildJsonArray(point.accelerations, allocator), allocator);
      point_obj.AddMember("effort", BuildJsonArray(point.effort, allocator), allocator);
      points.PushBack(point_obj, allocator);
    }
    doc.AddMember("points", points, allocator);

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);

    return buffer.GetString();
  }

  static std::string BuildJson(const trajectory_msgs::msg::JointTrajectoryPoint &point) {
    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();

    doc.AddMember("time_from_start", point.time_from_start.sec + point.time_from_start.nanosec / 1e9, allocator);
    doc.AddMember("positions", BuildJsonArray(point.positions, allocator), allocator);
    doc.AddMember("velocities", BuildJsonArray(point.velocities, allocator), allocator);
    doc.AddMember("accelerations", BuildJsonArray(point.accelerations, allocator), allocator);
    doc.AddMember("effort", BuildJsonArray(point.effort, allocator), allocator);

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);

    return buffer.GetString();
  }

  static std::string BuildJson(const std::vector<double> &vec) {
    rapidjson::Document doc;
    doc.SetArray();
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();
    for (const auto &val : vec) {
      doc.PushBack(val, allocator);
    }
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);
    return buffer.GetString();
  }
};

namespace BT {

///////////////////
// DESERIALIZERS //
///////////////////

template <>
inline std::vector<double> convertFromString(BT::StringView str) {
  rapidjson::Document doc;
  doc.Parse(str.data());
  std::vector<double> vec;

  if (doc.IsArray()) {
    for (const auto &val : doc.GetArray()) {
      if (val.IsDouble() || val.IsInt() || val.IsUint()) {
        vec.push_back(val.GetDouble());
      }
    }
  }

  return vec;
}

template <>
inline trajectory_msgs::msg::JointTrajectory convertFromString(BT::StringView str) {
  trajectory_msgs::msg::JointTrajectory trajectory;
  rapidjson::Document doc;
  doc.Parse(str.data());

  if (doc.HasMember("points") && doc["points"].IsArray()) {
    const auto &points = doc["points"].GetArray();
    for (const auto &point : points) {
      trajectory_msgs::msg::JointTrajectoryPoint point_obj;
      // time_from_start 作为double存储，需拆分为sec和nanosec
      if (point.HasMember("time_from_start") && point["time_from_start"].IsNumber()) {
        double t = point["time_from_start"].GetDouble();
        point_obj.time_from_start.sec = static_cast<int32_t>(t);
        point_obj.time_from_start.nanosec = static_cast<uint32_t>((t - point_obj.time_from_start.sec) * 1e9);
      }
      if (point.HasMember("positions") && point["positions"].IsArray()) {
        for (const auto &val : point["positions"].GetArray()) {
          point_obj.positions.push_back(val.GetDouble());
        }
      }
      if (point.HasMember("velocities") && point["velocities"].IsArray()) {
        for (const auto &val : point["velocities"].GetArray()) {
          point_obj.velocities.push_back(val.GetDouble());
        }
      }
      if (point.HasMember("accelerations") && point["accelerations"].IsArray()) {
        for (const auto &val : point["accelerations"].GetArray()) {
          point_obj.accelerations.push_back(val.GetDouble());
        }
      }
      if (point.HasMember("effort") && point["effort"].IsArray()) {
        for (const auto &val : point["effort"].GetArray()) {
          point_obj.effort.push_back(val.GetDouble());
        }
      }
      trajectory.points.push_back(point_obj);
    }
  } else {
    throw BT::RuntimeError("Invalid trajectory format");
  }

  return trajectory;
}

template <>
inline trajectory_msgs::msg::JointTrajectoryPoint convertFromString(BT::StringView str) {
  trajectory_msgs::msg::JointTrajectoryPoint point;
  rapidjson::Document doc;
  doc.Parse(str.data());

  if (doc.HasMember("time_from_start") && doc["time_from_start"].IsNumber()) {
    double t = doc["time_from_start"].GetDouble();
    point.time_from_start.sec = static_cast<int32_t>(t);
    point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);
  }
  if (doc.HasMember("positions") && doc["positions"].IsArray()) {
    for (const auto &val : doc["positions"].GetArray()) {
      point.positions.push_back(val.GetDouble());
    }
  }
  if (doc.HasMember("velocities") && doc["velocities"].IsArray()) {
    for (const auto &val : doc["velocities"].GetArray()) {
      point.velocities.push_back(val.GetDouble());
    }
  }
  if (doc.HasMember("accelerations") && doc["accelerations"].IsArray()) {
    for (const auto &val : doc["accelerations"].GetArray()) {
      point.accelerations.push_back(val.GetDouble());
    }
  }
  if (doc.HasMember("effort") && doc["effort"].IsArray()) {
    for (const auto &val : doc["effort"].GetArray()) {
      point.effort.push_back(val.GetDouble());
    }
  }

  return point;
}

}  // namespace BT

class MoveitHelper {
 public:
  static MoveitHelper &GetInstance() {
    static MoveitHelper instance;
    return instance;
  }

  moveit::planning_interface::MoveGroupInterface &GetMoveGroupInterface() {
    if (mgi_in_use_.exchange(true)) {
      throw std::runtime_error("MoveGroupInterface is already in use");
    }
    return mgi_;
  }

  bool ReleaseMoveGroupInterface() { return mgi_in_use_.exchange(false); }

  bool IsMoveGroupInterfaceInUse() const { return mgi_in_use_.load(); }

  void DrawTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) {
    RCLCPP_INFO(mgi_node_->get_logger(), "Drawing trajectory");
    mvt_.deleteAllMarkers();
    const auto *const jmg = mgi_.getRobotModel()->getJointModelGroup(mgi_.getName());
    const auto *eff_link_model = jmg->getLinkModel("end_yaw_2_link");

    mvt_.publishTrajectoryLine(trajectory, eff_link_model, jmg);
    mvt_.trigger();
  }

  void DrawTrajectory(const trajectory_msgs::msg::JointTrajectory &trajectory) {
    RCLCPP_INFO(mgi_node_->get_logger(), "Drawing trajectory");
    mvt_.deleteAllMarkers();
    const auto *const jmg = mgi_.getRobotModel()->getJointModelGroup(mgi_.getName());
    const auto *eff_link_model = jmg->getLinkModel("end_yaw_2_link");

    const auto traj_msg = [&] {
      moveit_msgs::msg::RobotTrajectory ret;
      ret.joint_trajectory = trajectory;
      return ret;
    }();
    mvt_.publishTrajectoryLine(traj_msg, eff_link_model, jmg);
    mvt_.trigger();
  }

 private:
  MoveitHelper()
      : mgi_node_{rclcpp::Node::make_shared("fluorite_behaviour_tree_mgi_node")},
        mgi_node_executor_{std::make_shared<rclcpp::executors::SingleThreadedExecutor>()},
        mgi_(mgi_node_, "fluorite_arm"),
        mvt_{mgi_node_, "base_yaw_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, mgi_.getRobotModel()} {
    mgi_node_executor_->add_node(mgi_node_);
    mgi_node_thread_ = std::thread([&] { mgi_node_executor_->spin(); });
    mvt_.deleteAllMarkers();
    mvt_.loadRemoteControl();
    RCLCPP_INFO(mgi_node_->get_logger(), "Planning frame: %s, Joint model group: %s", mgi_.getPlanningFrame().c_str(),
                mgi_.getName().c_str());
  };
  ~MoveitHelper() {
    mgi_node_executor_->cancel();
    mgi_node_executor_->remove_node(mgi_node_);
    mgi_node_thread_.join();
    mgi_node_.reset();
    mgi_node_executor_.reset();
  }

  MoveitHelper(const MoveitHelper &) = delete;
  MoveitHelper &operator=(const MoveitHelper &) = delete;
  MoveitHelper(MoveitHelper &&) = delete;
  MoveitHelper &operator=(MoveitHelper &&) = delete;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> instance_;
  rclcpp::Node::SharedPtr mgi_node_;
  rclcpp::Executor::SharedPtr mgi_node_executor_;
  std::thread mgi_node_thread_;
  moveit::planning_interface::MoveGroupInterface mgi_;
  moveit_visual_tools::MoveItVisualTools mvt_;
  std::atomic<bool> mgi_in_use_{false};
};

#endif  // MOVEIT_HPP
