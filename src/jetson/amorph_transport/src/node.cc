
#include "node.hpp"

#include "host_comm.hpp"
#include "logger.hpp"

namespace amorph {

using namespace text::literals;

TransportNode::TransportNode(const rclcpp::NodeOptions &options)
    : Node("arm_controller", rclcpp::NodeOptions()                   //
                                 .allow_undeclared_parameters(true)  //
                                 .automatically_declare_parameters_from_overrides(true)) {
  params_manager_.Init();
  InitComm();
  InitActionServers();
  L::I("node initialized."_green);
}

void TransportNode::InitComm() {
  const std::string can_port = params_manager_.data().rpc.can_port;
  const int mosi_frame_id = params_manager_.data().rpc.mosi_frame_id;
  const int miso_frame_id = params_manager_.data().rpc.miso_frame_id;

  comm_can_ = std::make_unique<rm::hal::Can>(can_port.c_str());
  stm32_ = std::make_unique<Stm32>(*comm_can_, miso_frame_id);

  static rm::device::DeviceManager<1> stm32_connection_watcher;
  stm32_connection_watcher << stm32_.get();
  stm32_connection_watcher.OnDeviceStatusChange([](rm::device::Device *device) {
    if (device->online_status() == rm::device::Device::kOk) {
      L::I("STM32 connection established."_green);
    } else {
      L::W("STM32 connection lost."_yellow);
    }
  });

  // 订阅关节位置，发布JointState
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(  //
      params_manager_.data().joint_state_topic,                       //
      10                                                              //
  );
  static sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.name = params_manager_.data().joint_names;
  stm32_->OnRx([&](const Stm32Feedback &data) {
    std::lock_guard<std::mutex> lock(stm32_mutex_);
    joint_state_msg.position.clear();
    joint_state_msg.position.reserve(6);
    for (size_t i = 0; i < 6; ++i) {
      joint_state_msg.position.push_back(data.joint_positions[i]);
    }
    joint_state_msg.position.push_back(0.f);
    joint_state_msg.velocity.clear();
    joint_state_msg.velocity.reserve(6);
    for (size_t i = 0; i < 6; ++i) {
      joint_state_msg.velocity.push_back(data.joint_velocities[i]);
    }
    joint_state_msg.velocity.push_back(0.f);
    joint_state_msg.effort.clear();
    joint_state_msg.effort.resize(7, 0.0);
    joint_state_msg.header.stamp = rclcpp::Clock().now();
    joint_state_pub_->publish(joint_state_msg);
  });

  comm_can_->Begin();

  joint_command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/transport/joint_command", 10);

  stm32_tx_timer_ = create_wall_timer(std::chrono::milliseconds(5), [&]() {
    stm32_connection_watcher.Update();

    std::lock_guard<std::mutex> lock(stm32_mutex_);
    // 更新轨迹跟踪器，将插值结果写入 STM32 发送缓冲区
    trajectory_tracker_.Update();
    if (trajectory_tracker_.is_initialized()) {
      const auto pt = trajectory_tracker_.current_point();
      std_msgs::msg::Float64MultiArray cmd_msg;
      cmd_msg.data.resize(6);
      for (size_t i = 0; i < 6; ++i) {
        stm32_->tx_data_.target_joint_positions[i] = static_cast<float>(pt.positions[i]);
        cmd_msg.data[i] = pt.positions[i];
      }
      joint_command_pub_->publish(cmd_msg);
    }

    stm32_->Write();
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
  execute_task_solution_server_ = rclcpp_action::create_server<ExecuteTaskSolution>(  //
      this,                                                                            //
      "execute_task_solution",                                                        //
      [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ExecuteTaskSolution::Goal> goal) {
        L::I("Received execute_task_solution goal");
        (void)uuid;  // unused
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandleExecuteTaskSolution> goal_handle) {
        L::I("Received execute_task_solution cancel request");
        (void)goal_handle;  // unused
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleExecuteTaskSolution> goal_handle) {
        std::thread{
            [this](const std::shared_ptr<GoalHandleExecuteTaskSolution> goal_handle) { this->Execute(goal_handle); },
            goal_handle}
            .detach();
      });
  L::I("Action servers ok."_green);
}

void TransportNode::Execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
  L::I("Executing arm trajectory goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  feedback->joint_names = params_manager_.data().joint_names;

  const auto &trajectory = goal->trajectory;

  // 检查 STM32 当前模式是否允许接管
  Stm32Feedback::Mode stm32_mode;
  {
    std::lock_guard<std::mutex> lock(stm32_mutex_);
    stm32_mode = stm32_->rx_data().mode;
  }
  if (stm32_mode != Stm32Feedback::kIdle && stm32_mode != Stm32Feedback::kHostControl) {
    L::W("STM32 not in idle/host-control mode (current: %d), aborting trajectory!", static_cast<int>(stm32_mode));
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = "STM32 not ready for host control";
    goal_handle->abort(result);
    return;
  }

  // 将轨迹加载到 tracker（内部会做合法性校验和限位）
  trajectory_tracker_.SwitchReferenceTrajectory(trajectory);

  // 请求 STM32 进入上位机控制模式
  {
    std::lock_guard<std::mutex> lock(stm32_mutex_);
    stm32_->tx_data_.take_control = true;
  }

  // 等待轨迹执行完毕或被取消
  rclcpp::Rate feedback_rate(50);  // 50Hz 反馈频率
  while (rclcpp::ok()) {
    // 检查是否收到取消请求
    if (goal_handle->is_canceling()) {
      {
        std::lock_guard<std::mutex> lock(stm32_mutex_);
        stm32_->tx_data_.take_control = false;
      }
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "Trajectory canceled by client";
      goal_handle->canceled(result);
      L::I("Arm trajectory goal canceled."_yellow);
      return;
    }

    // 检查 STM32 是否出现故障
    Stm32Feedback::Mode current_mode;
    {
      std::lock_guard<std::mutex> lock(stm32_mutex_);
      current_mode = stm32_->rx_data().mode;
    }
    if (current_mode == Stm32Feedback::kFault) {
      {
        std::lock_guard<std::mutex> lock(stm32_mutex_);
        stm32_->tx_data_.take_control = false;
      }
      result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      result->error_string = "STM32 entered fault state during execution";
      goal_handle->abort(result);
      L::E("STM32 fault during trajectory execution!");
      return;
    }

    // 构造并发布反馈
    feedback->desired = trajectory_tracker_.current_point();
    {
      std::lock_guard<std::mutex> lock(stm32_mutex_);
      const auto &rx = stm32_->rx_data();
      feedback->actual.positions.resize(6);
      feedback->actual.velocities.resize(6);
      for (size_t i = 0; i < 6; ++i) {
        feedback->actual.positions[i] = rx.joint_positions[i];
        feedback->actual.velocities[i] = rx.joint_velocities[i];
      }
    }
    feedback->error.positions.resize(6);
    for (size_t i = 0; i < 6; ++i) {
      feedback->error.positions[i] = feedback->desired.positions[i] - feedback->actual.positions[i];
    }
    goal_handle->publish_feedback(feedback);

    // 检查轨迹是否执行完毕
    if (trajectory_tracker_.finished()) {
      break;
    }

    feedback_rate.sleep();
  }

  // 轨迹执行完毕，释放控制权
  {
    std::lock_guard<std::mutex> lock(stm32_mutex_);
    stm32_->tx_data_.take_control = false;
  }
  result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
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

void TransportNode::Execute(const std::shared_ptr<GoalHandleExecuteTaskSolution> goal_handle) {
  L::I("Executing task solution goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ExecuteTaskSolution::Result>();
  auto feedback = std::make_shared<ExecuteTaskSolution::Feedback>();

  const auto &solution = goal->solution;
  L::I("Task solution contains %zu sub-trajectories", solution.sub_trajectory.size());

  // 遍历所有子轨迹
  for (size_t i = 0; i < solution.sub_trajectory.size(); ++i) {
    if (goal_handle->is_canceling()) {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
      goal_handle->canceled(result);
      L::I("Task solution execution canceled."_yellow);
      return;
    }

    const auto &sub_traj = solution.sub_trajectory[i];
    const auto &exec_info = sub_traj.execution_info;
    const auto &joint_traj = sub_traj.trajectory.joint_trajectory;

    // 获取控制器名称
    std::string controller_name;
    if (!exec_info.controller_names.empty()) {
      controller_name = exec_info.controller_names[0];
    }

    L::I("Executing sub-trajectory %zu/%zu for controller: %s", i + 1, solution.sub_trajectory.size(),
         controller_name.c_str());

    // 跳过空轨迹（如 ModifyPlanningScene 等不产生运动的 stage）
    if (joint_traj.points.empty()) {
      L::I("Empty trajectory, skipping");
      feedback->sub_id = static_cast<uint32_t>(i);
      feedback->sub_no = static_cast<uint32_t>(solution.sub_trajectory.size());
      goal_handle->publish_feedback(feedback);
      continue;
    }

    // 根据 controller 名称判断是机械臂还是夹爪
    if (controller_name.find("arm") != std::string::npos ||
        controller_name.find("follow_joint_trajectory") != std::string::npos) {
      // 这是机械臂轨迹，检查关节数量是否匹配
      if (!joint_traj.joint_names.empty() && joint_traj.joint_names.size() != 6) {
        L::W("Arm trajectory has %zu joints, expected 6, skipping", joint_traj.joint_names.size());
        feedback->sub_id = static_cast<uint32_t>(i);
        feedback->sub_no = static_cast<uint32_t>(solution.sub_trajectory.size());
        goal_handle->publish_feedback(feedback);
        continue;
      }

      auto arm_goal = std::make_shared<FollowJointTrajectory::Goal>();
      arm_goal->trajectory = joint_traj;

      // 检查 STM32 状态
      Stm32Feedback::Mode stm32_mode;
      {
        std::lock_guard<std::mutex> lock(stm32_mutex_);
        stm32_mode = stm32_->rx_data().mode;
      }
      if (stm32_mode != Stm32Feedback::kIdle && stm32_mode != Stm32Feedback::kHostControl) {
        L::W("STM32 not in idle/host-control mode (current: %d), aborting task!", static_cast<int>(stm32_mode));
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
        goal_handle->abort(result);
        return;
      }

      // 加载轨迹
      {
        std::lock_guard<std::mutex> lock(stm32_mutex_);
        trajectory_tracker_.SwitchReferenceTrajectory(arm_goal->trajectory);
        stm32_->tx_data_.take_control = true;
      }

      // 等待轨迹执行完成
      rclcpp::Rate rate(50);
      while (rclcpp::ok() && !trajectory_tracker_.finished()) {
        if (goal_handle->is_canceling()) {
          std::lock_guard<std::mutex> lock(stm32_mutex_);
          stm32_->tx_data_.take_control = false;
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
          goal_handle->canceled(result);
          L::I("Task solution execution canceled."_yellow);
          return;
        }

        Stm32Feedback::Mode current_mode;
        {
          std::lock_guard<std::mutex> lock(stm32_mutex_);
          current_mode = stm32_->rx_data().mode;
        }
        if (current_mode == Stm32Feedback::kFault) {
          std::lock_guard<std::mutex> lock(stm32_mutex_);
          stm32_->tx_data_.take_control = false;
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
          goal_handle->abort(result);
          L::E("STM32 fault during task execution!");
          return;
        }

        rate.sleep();
      }

      {
        std::lock_guard<std::mutex> lock(stm32_mutex_);
        stm32_->tx_data_.take_control = false;
      }

    } else if (controller_name.find("gripper") != std::string::npos) {
      // 这是夹爪轨迹
      L::I("Gripper trajectory execution - TODO");
      // TODO: 实现夹爪控制
      
      // 更新反馈
      feedback->sub_id = static_cast<uint32_t>(i);
      feedback->sub_no = static_cast<uint32_t>(solution.sub_trajectory.size());
      goal_handle->publish_feedback(feedback);
    } else if (controller_name.empty()) {
      // 空控制器名称，可能是 ModifyPlanningScene 等 stage，已在前面过滤空轨迹时处理
      L::I("No controller specified, skipping (likely a planning scene modification)");
      
      // 更新反馈
      feedback->sub_id = static_cast<uint32_t>(i);
      feedback->sub_no = static_cast<uint32_t>(solution.sub_trajectory.size());
      goal_handle->publish_feedback(feedback);
    } else {
      L::W("Unknown controller type: %s, skipping", controller_name.c_str());
      
      // 更新反馈
      feedback->sub_id = static_cast<uint32_t>(i);
      feedback->sub_no = static_cast<uint32_t>(solution.sub_trajectory.size());
      goal_handle->publish_feedback(feedback);
    }
  }

  result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  goal_handle->succeed(result);
  L::I("Task solution executed successfully."_green);
}

}  // namespace amorph

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(amorph::TransportNode)
