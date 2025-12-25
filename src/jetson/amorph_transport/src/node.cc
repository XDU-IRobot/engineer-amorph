
#include "node.hpp"

#include "logger.hpp"
#include "rpc/typedefs.hpp"

namespace amorph {

using namespace text::literals;

TransportNode::TransportNode(const rclcpp::NodeOptions &options) : Node("arm_controller", options) {
  params_manager_.Init();
  InitComm();
  InitActionServers();

  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(  //
      params_manager_.data().joint_state_topic,                       //
      10                                                              //
  );

  L::I("node initialized."_green);
}

void TransportNode::InitComm() {
  const std::string can_port = params_manager_.data().rpc.can_port;
  const int mosi_frame_id = params_manager_.data().rpc.mosi_frame_id;
  const int miso_frame_id = params_manager_.data().rpc.miso_frame_id;

  static rm::modules::SparseValueWatcher<bool, true> rpc_connection_watcher;
  rpc_connection_watcher.OnValueChange([](const bool &old_value, const bool &new_value) {
    (void)old_value;  // unused
    if (new_value) {
      L::I("RPC connection established."_green);
    } else {
      L::W("RPC connection lost."_yellow);
    }
  });

  comm_can_ = std::make_unique<rm::hal::Can>(can_port.c_str());
  auto fdcan_transport = std::make_shared<FdCanTransport>(*comm_can_, miso_frame_id, mosi_frame_id);
  stm32_.AttachToConnection(fdcan_transport);
  comm_can_->Begin();

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

  // 订阅关节位置，发布JointState
  static sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->now();
  joint_state_msg.name = params_manager_.data().joint_names;
  stm32_.rpc()->subscribe("joint_position", [this](const JointPosition<6> &msg) {
    joint_state_msg.position.assign(msg.positions.begin(), msg.positions.end());
    joint_state_pub_->publish(joint_state_msg);
  });

  // rpc心跳定时器
  rpc_heartbeat_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
    rpc_connection_watcher.Update(stm32_.Connected());
    stm32_.KeepConnection();
  });

  // 订阅自定义控制器数据，转发到CAN总线上
  custom_controller_ = std::make_unique<CustomController>(*comm_can_);
  custom_ctrl_sub_ = create_subscription<rm_referee_msgs::msg::CustomRobotData>(  //
      "rm_referee/custom_robot_data",                                             //
      rclcpp::SensorDataQoS(),                                                    //
      [this](const rm_referee_msgs::msg::CustomRobotData::SharedPtr msg) {
        memcpy(&custom_controller_->data, msg->data.data(), sizeof(custom_controller_->data));
        custom_controller_->Write();
      });

  L::I("Comm with stm32 ok."_green);
}

void TransportNode::InitActionServers() {
  arm_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(  //
      this,                                                                  //
      params_manager_.data().arm_action_server_name,                         //
      [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
        L::I("Received arm trajectory goal");
        (void)uuid;  // unused
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
        L::I("Received arm trajectory cancel request");
        (void)goal_handle;  // unused
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
        std::thread{
            [this](const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) { this->Execute(goal_handle); },
            goal_handle}
            .detach();
      });
  gripper_action_server_ = rclcpp_action::create_server<GripperCommand>(  //
      this,                                                               //
      params_manager_.data().gripper_action_server_name,                  //
      [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GripperCommand::Goal> goal) {
        L::I("Received gripper command goal");
        (void)uuid;  // unused
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandleGripperCommand> goal_handle) {
        L::I("Received gripper command cancel request");
        (void)goal_handle;  // unused
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleGripperCommand> goal_handle) {
        std::thread{[this](const std::shared_ptr<GoalHandleGripperCommand> goal_handle) { this->Execute(goal_handle); },
                    goal_handle}
            .detach();
      });
  L::I("Action servers ok."_green);
}

void TransportNode::Execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
  L::I("Executing arm trajectory goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<FollowJointTrajectory::Result>();

  // 把ROS的轨迹消息转换为RPC的轨迹消息
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
    jt_point.time_from_start = rclcpp::Duration(point.time_from_start).to_chrono<decltype(jt_point.time_from_start)>();
    traj.points.push_back(jt_point);
  }

  // 发送执行请求
  JointTrajectory exec_req;
  exec_req.points = traj.points;
  // stm32_.rpc()->cmd("execution_request")->msg(exec_req)->call();

  // TODO: 处理执行反馈
  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  goal_handle->publish_feedback(feedback);

  goal_handle->succeed(result);
  L::I("Arm trajectory goal executed."_green);
}

void TransportNode::Execute(const std::shared_ptr<GoalHandleGripperCommand> goal_handle) {
  L::I("Executing gripper command goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<GripperCommand::Result>();
  L::I("Gripper command: position=%f, max_effort=%f", goal->command.position, goal->command.max_effort);

  // TODO

  L::I("Gripper command goal executed."_green);
}

}  // namespace amorph

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(amorph::TransportNode)
