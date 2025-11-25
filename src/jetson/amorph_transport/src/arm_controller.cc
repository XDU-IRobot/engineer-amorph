
#include "arm_controller.hpp"

#include <cstdint>
#include <memory>
#include <thread>
#include <chrono>

#include <librm.hpp>

#include "logger.hpp"

namespace amorph {

using namespace text::literals;

ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions &options) : Node("arm_controller", options) {
  using namespace std::placeholders;

  declare_parameter<std::string>("rpc.can_port", "can0");
  declare_parameter<int>("rpc.mosi_frame_id", 0x23);
  declare_parameter<int>("rpc.miso_frame_id", 0x33);

  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(  //
      this,                                                              //
      "~/follow_joint_trajectory",                                       //
      std::bind(&ArmControllerNode::handle_goal, this, _1, _2),          //
      std::bind(&ArmControllerNode::handle_cancel, this, _1),            //
      std::bind(&ArmControllerNode::handle_accepted, this, _1));

  L::I("node initialized."_green);
}

void ArmControllerNode::InitRpc() {
  std::string can_port = get_parameter("rpc.can_port").as_string();
  int mosi_frame_id = get_parameter("rpc.mosi_frame_id").as_int();
  int miso_frame_id = get_parameter("rpc.miso_frame_id").as_int();

  static rm::hal::Can rpc_can{can_port.c_str()};
  auto fdcan_transport = std::make_shared<FdCanTransport>(rpc_can, miso_frame_id, mosi_frame_id);
  stm32_.AttachToConnection(fdcan_transport);
  rpc_can.Begin();

  // 订阅日志消息
  stm32_.rpc()->subscribe("log_message", [this](const LogMessage &msg) {
    switch (msg.log_level) {
      case LogMessage::LogLevel::kDebug:
        L::D("[stm32]"_bold + " %s", msg.str.c_str());
        break;
      case LogMessage::LogLevel::kInfo:
        L::I("[stm32]"_bold + " %s", msg.str.c_str());
        break;
      case LogMessage::LogLevel::kWarn:
        L::W("[stm32]"_bold + " %s", msg.str.c_str());
        break;
      case LogMessage::LogLevel::kError:
        L::E("[stm32]"_bold + " %s", msg.str.c_str());
        break;
      case LogMessage::LogLevel::kFatal:
        L::F("[stm32]"_bold + " %s", msg.str.c_str());
        break;
      default:
        L::I("[stm32]"_bold + " [UNKNOWN LEVEL]"_yellow + " %s", msg.str.c_str());
        break;
    }
  });

  // rpc心跳定时器
  auto heartbeat_timer = create_wall_timer(std::chrono::milliseconds(300), [this]() { stm32_.KeepConnection(); });

  L::I("RPC with stm32 ok."_green);
}

rclcpp_action::GoalResponse ArmControllerNode::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                           std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
  L::I("Received goal request");
  (void)uuid;  // unused
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmControllerNode::handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
  L::I("Received cancel request");
  (void)goal_handle;  // unused
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmControllerNode::handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
  std::thread{std::bind(&ArmControllerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void ArmControllerNode::execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
  L::I("Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<FollowJointTrajectory::Result>();

  // 转换轨迹
  JointTrajectory traj;
  for (const auto &point : goal->trajectory.points) {
    JointTrajectoryPoint jt_point;
    jt_point.positions.clear();
    jt_point.positions.reserve(point.positions.size());
    for (const auto &v : point.positions) {
      jt_point.positions.push_back(v);
    }
    jt_point.velocities.clear();
    jt_point.velocities.reserve(point.velocities.size());
    for (const auto &v : point.velocities) {
      jt_point.velocities.push_back(v);
    }
    jt_point.accelerations.clear();
    jt_point.accelerations.reserve(point.accelerations.size());
    for (const auto &v : point.accelerations) {
      jt_point.accelerations.push_back(v);
    }
    jt_point.effort.clear();
    jt_point.effort.reserve(point.effort.size());
    for (const auto &v : point.effort) {
      jt_point.effort.push_back(v);
    }
    jt_point.time_from_start =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(point.time_from_start.sec) +
                                                              std::chrono::nanoseconds(point.time_from_start.nanosec));
    traj.points.push_back(jt_point);
  }

  // 发送执行请求
  ExecutionRequest exec_req;
  exec_req.trajectory = traj;
  stm32_.rpc()->cmd("execution_request")->msg(exec_req)->call();

  // TODO: 处理执行反馈
  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  goal_handle->publish_feedback(feedback);

  goal_handle->succeed(result);
  L::I("Goal succeeded"_green);
}

}  // namespace amorph
