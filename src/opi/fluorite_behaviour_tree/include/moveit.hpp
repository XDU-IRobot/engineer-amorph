/**
 * @file  moveit.hpp
 * @brief 本文件定义轨迹、点等等数据结构的json序列化和反序列化器，并且提供一个Moveit相关对象的单例供决策树节点使用
 */

#ifndef MOVEIT_HPP
#define MOVEIT_HPP

#include <memory>
#include <atomic>

#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
#include <rclcpp/logging.hpp>
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

 public:
  template <typename T>
  static std::string BuildJson(const std::vector<T> &vec) {
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

  static std::string BuildJson(const trajectory_msgs::msg::JointTrajectory &trajectory) {
    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();

    rapidjson::Value points(rapidjson::kArrayType);
    for (const auto &point : trajectory.points) {
      rapidjson::Value point_obj(rapidjson::kObjectType);
      point_obj.AddMember("time_from_start", point.time_from_start.sec + point.time_from_start.nanosec / 1e9,
                          allocator);

      // REMINDER
      // Value:如果字符串生命周期很短，那么必须用Value（复制一份字符串到allocator），如果用StringRef（传入字符串指针），变量销毁后则会访问到错误的地址。
      // StringRef:如果字符串变量生命周期能buffer.GetString之后，那么可以用StringRef（传入字符串指针）。

      point_obj.AddMember("positions", rapidjson::StringRef(BuildJson(point.positions).c_str()), allocator);
      point_obj.AddMember("velocities", rapidjson::StringRef(BuildJson(point.velocities).c_str()), allocator);
      point_obj.AddMember("accelerations", rapidjson::StringRef(BuildJson(point.accelerations).c_str()), allocator);
      point_obj.AddMember("effort", rapidjson::StringRef(BuildJson(point.effort).c_str()), allocator);
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
    doc.AddMember("positions", rapidjson::StringRef(BuildJson(point.positions).c_str()), allocator);
    doc.AddMember("velocities", rapidjson::StringRef(BuildJson(point.velocities).c_str()), allocator);
    doc.AddMember("accelerations", rapidjson::StringRef(BuildJson(point.accelerations).c_str()), allocator);
    doc.AddMember("effort", rapidjson::StringRef(BuildJson(point.effort).c_str()), allocator);

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
      if (val.IsDouble()) {
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
      point_obj.time_from_start.sec = point["time_from_start"].GetInt();
      point_obj.time_from_start.nanosec = point["time_from_start"].GetUint();
      point_obj.positions = convertFromString<std::vector<double>>(point["positions"].GetString());
      point_obj.velocities = convertFromString<std::vector<double>>(point["velocities"].GetString());
      point_obj.accelerations = convertFromString<std::vector<double>>(point["accelerations"].GetString());
      point_obj.effort = convertFromString<std::vector<double>>(point["effort"].GetString());
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

  point.positions = convertFromString<std::vector<double>>(doc["positions"].GetString());
  point.velocities = convertFromString<std::vector<double>>(doc["velocities"].GetString());
  point.accelerations = convertFromString<std::vector<double>>(doc["accelerations"].GetString());
  point.effort = convertFromString<std::vector<double>>(doc["effort"].GetString());

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

  bool ReleaseMoveGroupInterface() {
    if (!mgi_in_use_.exchange(false)) {
      return false;
    }
    return true;
  }

  bool IsMoveGroupInterfaceInUse() const { return mgi_in_use_.load(); }

  void DrawTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) {
    const auto jmg = mgi_.getRobotModel()->getJointModelGroup(mgi_.getName());
    RCLCPP_INFO(mgi_node_->get_logger(), "Drawing trajectory");
    mvt_.publishTrajectoryLine(trajectory, jmg);
    mvt_.trigger();
  }

  void DrawTrajectory(const trajectory_msgs::msg::JointTrajectory &trajectory) {
    const auto jmg = mgi_.getRobotModel()->getJointModelGroup(mgi_.getName());
    RCLCPP_INFO(mgi_node_->get_logger(), "Drawing trajectory");
    const auto traj_msg = [&] {
      moveit_msgs::msg::RobotTrajectory ret;
      ret.joint_trajectory = trajectory;
      return ret;
    }();
    mvt_.publishTrajectoryLine(traj_msg, jmg);
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