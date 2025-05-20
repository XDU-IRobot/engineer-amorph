/**
 * @file  arm/fsm.cc
 * @brief 状态机实现
 */

#include "arm/fsm.hpp"

#include <chrono>
#include <iostream>
#include <unordered_map>

#include <Eigen/Dense>

#include "arm/ros_params.hpp"
#include "keyboard_msgs/msg/key.hpp"
#include "tinyfsm.hpp"

namespace fluorite::arm {

// 这几个全局变量用于ROS节点和下面的几个状态机通信，extern声明在arm/global.hpp里
fluorite_msgs::msg::ArmState state_msg{};
std::unique_ptr<Actuator> actuator{nullptr};
std::unique_ptr<Controller> controller{nullptr};
RosParams ros_params;

namespace fsm {

// 忽略不关心的event
// these functions should never be called so wtf in them
void Arm::react(tinyfsm::Event const &event) { error("FSM: Arm received unknown event"); }
void GimbalBase::react(tinyfsm::Event const &event) { error("FSM: GimbalBase received unknown event"); }
void PneumaticSystem::react(tinyfsm::Event const &event) { error("FSM: PneumaticSystem received unknown event"); }

void Arm::EnableActuators(bool enable) {
  const auto dm_enable = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
  actuator->yaw.SendInstruction(dm_enable);
  actuator->pitch_1.SendInstruction(dm_enable);
  actuator->pitch_2.SendInstruction(dm_enable);
  actuator->end_yaw_1.SendInstruction(dm_enable);
  actuator->end_pitch.SendInstruction(dm_enable);
}

void Arm::CalcPidAndExecute() {
  const std::array<double, 6> kCurrentPosition = {state_msg.yaw_encoder,       state_msg.pitch_1_encoder,
                                                  state_msg.pitch_2_encoder,   state_msg.end_yaw_1_encoder,
                                                  state_msg.end_pitch_encoder, state_msg.end_yaw_2_encoder};
  const std::array<double, 6> kCurrentSpeed = {state_msg.yaw_encoder_speed,       state_msg.pitch_1_encoder_speed,
                                               state_msg.pitch_2_encoder_speed,   state_msg.end_yaw_1_encoder_speed,
                                               state_msg.end_pitch_encoder_speed, state_msg.end_yaw_2_encoder_speed};

  controller->trajectory_tracker.Update(kCurrentPosition, kCurrentSpeed);

  // 重力补偿
  const auto kCompensateTorques = controller->dynamic_model(
      {state_msg.yaw_encoder, state_msg.pitch_1_encoder, state_msg.pitch_2_encoder},
      {state_msg.yaw_encoder_speed, state_msg.pitch_1_encoder_speed, state_msg.pitch_2_encoder_speed}, {0.0, 0.0, 0.0},
      false);

  // apply output torque
  const auto kOutputTorque = controller->trajectory_tracker.output_torque();
  const auto kDesiredPosition = controller->trajectory_tracker.current_point().positions;
  actuator->yaw.SetPosition(kDesiredPosition[0], 0, 0, 90, 35);
  actuator->pitch_1.SetPosition(
      (kDesiredPosition[1] + ros_params.pitch_1_encoder_zero_offset) * 1.5,  // 传动链条 减速比2:3
      0, 0.65 * kCompensateTorques[1], 85, 35);
  actuator->pitch_2.SetPosition(kDesiredPosition[2], 0, 0.65 * kCompensateTorques[2], 70, 30);
  actuator->end_yaw_1.SetPosition(kDesiredPosition[3], 0, 0, 70, 30);
  actuator->end_pitch.SetPosition(kDesiredPosition[4], 0, 0, 70, 30);
  actuator->end_yaw_2.SetCurrent(kOutputTorque[5]);
}

namespace state {

namespace arm {

void Debug::entry() {
  Arm::EnableActuators(true);
  const auto kCurrentJointPositions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.velocities.resize(6);
    p.positions = {state_msg.yaw_encoder,       state_msg.pitch_1_encoder,   state_msg.pitch_2_encoder,
                   state_msg.end_yaw_1_encoder, state_msg.end_pitch_encoder, state_msg.end_yaw_2_encoder};
    return p;
  }();
  controller->trajectory_tracker.SwitchReferenceTrajectory(kCurrentJointPositions);
}
void KeyboardManual::entry() {
  Arm::EnableActuators(true);
  const auto kCurrentJointPositions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.resize(6);
    p.velocities.resize(6);
    p.positions = {state_msg.yaw_encoder,       state_msg.pitch_1_encoder,   state_msg.pitch_2_encoder,
                   state_msg.end_yaw_1_encoder, state_msg.end_pitch_encoder, state_msg.end_yaw_2_encoder};
    return p;
  }();
  controller->trajectory_tracker.SwitchReferenceTrajectory(kCurrentJointPositions);
}
void CustomCtrlManual::entry() {
  Arm::EnableActuators(true);
  const auto kCurrentJointPositions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.velocities.resize(6);
    p.positions = {state_msg.yaw_encoder,       state_msg.pitch_1_encoder,   state_msg.pitch_2_encoder,
                   state_msg.end_yaw_1_encoder, state_msg.end_pitch_encoder, state_msg.end_yaw_2_encoder};
    return p;
  }();
  controller->trajectory_tracker.SwitchReferenceTrajectory(kCurrentJointPositions);
}
void BehaviourTreeIdle::entry() {
  Arm::EnableActuators(true);
  const std::array<double, 6> kCurrentPosition = {state_msg.yaw_encoder,       state_msg.pitch_1_encoder,
                                                  state_msg.pitch_2_encoder,   state_msg.end_yaw_1_encoder,
                                                  state_msg.end_pitch_encoder, state_msg.end_yaw_2_encoder};
  const std::array<double, 6> kCurrentSpeed = {state_msg.yaw_encoder_speed,       state_msg.pitch_1_encoder_speed,
                                               state_msg.pitch_2_encoder_speed,   state_msg.end_yaw_1_encoder_speed,
                                               state_msg.end_pitch_encoder_speed, state_msg.end_yaw_2_encoder_speed};

  controller->trajectory_tracker.Update(kCurrentPosition, kCurrentSpeed);
  controller->trajectory_tracker.SwitchReferenceTrajectory([&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.velocities.resize(6);
    p.positions = {state_msg.yaw_encoder,       state_msg.pitch_1_encoder,   state_msg.pitch_2_encoder,
                   state_msg.end_yaw_1_encoder, state_msg.end_pitch_encoder, state_msg.end_yaw_2_encoder};
    return p;
  }());
}
std::function<void()> BehaviourTreeExecuting::execution_finished_callback_;
std::function<void()> BehaviourTreeExecuting::execution_interrupted_callback_;
void BehaviourTreeExecuting::entry() { Arm::EnableActuators(true); }
void Retracted::entry() {
  Arm::EnableActuators(true);
  const auto current_joint_positions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.velocities.resize(6);
    p.positions = {state_msg.yaw_encoder,       state_msg.pitch_1_encoder,   state_msg.pitch_2_encoder,
                   state_msg.end_yaw_1_encoder, state_msg.end_pitch_encoder, state_msg.end_yaw_2_encoder};
    return p;
  }();
  const auto retracted_joint_positions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {0.0, 0.1, 0.1, 0, 0, 0};
    p.velocities = {0, 0, 0, 0, 0, 0};
    return p;
  }();
  const auto interpolated_trajectory =
      controller->joint_space_interpolator.Interpolate(current_joint_positions, retracted_joint_positions,
                                                       std::chrono::milliseconds(1000), std::chrono::milliseconds(50));

  controller->trajectory_tracker.SwitchReferenceTrajectory(interpolated_trajectory, 1.0, true);
}
void NoForce::entry() {
  Arm::EnableActuators(false);
  controller->trajectory_tracker.ClearPid();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////CONTROL LOOPS///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Debug::react(event::Control const &control) {
  auto joint_positions = controller->trajectory_tracker.current_point();
  if (control.se == kRcSwitchUp) {
    joint_positions.positions[0] += control.right_y * 0.0017;
  } else if (control.se == kRcSwitchMiddle) {
    joint_positions.positions[1] += control.right_y * 0.0017;
  } else {
    if (control.sg == kRcSwitchUp) {
      joint_positions.positions[2] += control.right_y * 0.0017;
    } else if (control.sg == kRcSwitchMiddle) {
      joint_positions.positions[3] += control.right_y * 0.0017;
    } else {
      if (control.sh == kRcSwitchUp) {
        joint_positions.positions[4] += control.right_y * 0.0017;
      } else if (control.sh == kRcSwitchDown) {
        joint_positions.positions[5] += control.right_y * 0.0017;
      }
    }
  }
  controller->trajectory_tracker.SwitchReferenceTrajectory(joint_positions);
  Arm::CalcPidAndExecute();

  // state transition
  if (control.sd != kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<NoForce>{});
    return;
  }
  if (control.sf == kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<Debug>{});
    return;
  }
  switch (control.sc) {
    case kRcSwitchUp: {
      DispatchEvent(event::SwitchMode<BehaviourTreeIdle>{});
      return;
    }
    case kRcSwitchMiddle: {
      DispatchEvent(event::SwitchMode<KeyboardManual>{});
      return;
    }
    case kRcSwitchDown: {
      DispatchEvent(event::SwitchMode<Retracted>{});
      return;
    }
  }
}
void KeyboardManual::react(event::Control const &control) {
  // 如果键盘有输入，
  if (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_F) ||
      control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_V) ||
      control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_C) ||
      control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_B) ||
      control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_Z) ||
      control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_X)) {
    // 根据键盘输入设置目标末端位姿
    double delta_x{0}, delta_y{0}, delta_z{0}, delta_pitch{0}, delta_roll{0};
    if (control.keyboard_key.at(keyboard_msgs::msg::Key::MODIFIER_LCTRL)) {
      delta_roll = (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_F) -
                    control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_V)) *
                   0.0013;
      delta_pitch = (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_C) -
                     control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_B)) *
                    0.0013;
    } else {
      if (control.keyboard_key.at(keyboard_msgs::msg::Key::MODIFIER_LSHIFT)) {
        delta_z = (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_F) -
                   control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_V)) *
                  0.0005;
      } else {
        delta_y = (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_V) -
                   control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_F)) *
                  0.0005;
      }
      delta_x = (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_C) -
                 control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_B)) *
                0.0005;
    }

    const double delta_yaw = (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_Z) -
                              control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_X)) *
                             0.0013;

    Eigen::Quaterniond q(
        state_msg.target_end_effector_pose.orientation.w, state_msg.target_end_effector_pose.orientation.x,
        state_msg.target_end_effector_pose.orientation.y, state_msg.target_end_effector_pose.orientation.z);
    const Eigen::AngleAxisd yaw_angle(delta_yaw, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd pitch_angle(delta_pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd roll_angle(delta_roll, Eigen::Vector3d::UnitX());
    q = q * yaw_angle * pitch_angle * roll_angle;

    state_msg.target_end_effector_pose.position.x += delta_x;
    state_msg.target_end_effector_pose.position.y += delta_y;
    state_msg.target_end_effector_pose.position.z += delta_z;
    state_msg.target_end_effector_pose.orientation.x = q.x();
    state_msg.target_end_effector_pose.orientation.y = q.y();
    state_msg.target_end_effector_pose.orientation.z = q.z();
    state_msg.target_end_effector_pose.orientation.w = q.w();

    // 如果上一轮计算结束（锁被释放），才计算一次新的目标关节位置，否则还是按TrajectoryTracker中的参考轨迹继续运动
    if (controller->ik_calculation_mutex.try_lock()) {
      controller->ik_calculation_thread_pool_.enqueue([&] {  // 异步计算ik，不阻塞控制循环
        const auto [kSolved, kIKResult] = controller->moveit_helper.CalculateIK(state_msg.target_end_effector_pose);
        if (kSolved) {
          controller->trajectory_tracker.SwitchReferenceTrajectory(kIKResult);
          state_msg.last_ik_success = true;
        } else {
          state_msg.last_ik_success = false;
        }

        controller->ik_calculation_mutex.unlock();  // 计算完毕，解锁
      });
    }
  }

  Arm::CalcPidAndExecute();

  // state transition
  if (control.sd != kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<NoForce>{});
    return;
  }
  if (control.sf == kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<Debug>{});
    return;
  }
  switch (control.sc) {
    case kRcSwitchUp: {
      DispatchEvent(event::SwitchMode<BehaviourTreeIdle>{});
      return;
    }
    case kRcSwitchMiddle: {
      DispatchEvent(event::SwitchMode<KeyboardManual>{});
      return;
    }
    case kRcSwitchDown: {
      DispatchEvent(event::SwitchMode<Retracted>{});
      return;
    }
  }
}
void CustomCtrlManual::react(event::Control const &control) {
  if constexpr (std::is_same_v<decltype(control.custom_controller_command), custom_controller::JointCommand>) {
    if (control.custom_controller_command.ok) {  // 这里不ok说明自定义控制器那边有问题，那就不接受传来的命令
      const auto target_joint_positions = [&] {
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions.resize(6);
        p.velocities = {0, 0, 0, 0, 0, 0};
        p.positions[0] = control.custom_controller_command.joint_positions[0];
        p.positions[1] = control.custom_controller_command.joint_positions[1];
        p.positions[2] = control.custom_controller_command.joint_positions[2];
        p.positions[3] = control.custom_controller_command.joint_positions[3];
        p.positions[4] = control.custom_controller_command.joint_positions[4];
        p.positions[5] = control.custom_controller_command.joint_positions[5];
        return p;
      }();
      controller->trajectory_tracker.SwitchReferenceTrajectory(target_joint_positions);
    }
  } else if constexpr (std::is_same_v<decltype(control.custom_controller_command),
                                      custom_controller::CartesianCommand>) {
    // TODO: 感觉T265自定义控制器不会再用了
  }
  Arm::CalcPidAndExecute();

  // state transition
  if (control.sd != kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<NoForce>{});
    return;
  }
  if (control.sf == kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<Debug>{});
    return;
  }
  switch (control.sc) {
    case kRcSwitchUp: {
      if (controller->hotkeys.toggle_custom_ctrl.have_pressed()) {
        DispatchEvent(event::SwitchMode<BehaviourTreeIdle>{});
      }
      return;
    }
    case kRcSwitchMiddle: {
      DispatchEvent(event::SwitchMode<KeyboardManual>{});
      return;
    }
    case kRcSwitchDown: {
      DispatchEvent(event::SwitchMode<Retracted>{});
      return;
    }
  }
}
void BehaviourTreeIdle::react(event::Control const &control) {
  if (control.mouse_wheel != 0) {
    const auto target_joint_positions = [&] {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.velocities.resize(6);
      p.positions = {state_msg.yaw_encoder,       state_msg.pitch_1_encoder + control.mouse_wheel * 0.033,
                     state_msg.pitch_2_encoder,   state_msg.end_yaw_1_encoder,
                     state_msg.end_pitch_encoder, state_msg.end_yaw_2_encoder};
      return p;
    }();
    controller->trajectory_tracker.SwitchReferenceTrajectory(target_joint_positions);
  }

  Arm::CalcPidAndExecute();

  // state transition
  if (control.sd != kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<NoForce>{});
    return;
  }
  if (control.sf == kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<Debug>{});
    return;
  }
  switch (control.sc) {
    case kRcSwitchUp: {
      if (controller->hotkeys.toggle_custom_ctrl.have_pressed()) {
        DispatchEvent(event::SwitchMode<CustomCtrlManual>{});
      }
      return;
    }
    case kRcSwitchMiddle: {
      DispatchEvent(event::SwitchMode<KeyboardManual>{});
      return;
    }
    case kRcSwitchDown: {
      DispatchEvent(event::SwitchMode<Retracted>{});
      return;
    }
  }
}
void BehaviourTreeIdle::react(
    event::BehaviourTreeCommand<fluorite_msgs::action::ArmEndEffectorCartesianCommand::Goal> const &e) {
  controller->ik_calculation_thread_pool_.enqueue([&] {  // 异步计算ik，不阻塞控制循环
    controller->ik_calculation_mutex.lock();             // 阻塞等待上一次ik计算完成
    // 尝试逆解，成功的话就把解出来的关节位置送进轨迹跟踪器
    state_msg.target_end_effector_pose = e.command.goal_end_effector_pose;
    const auto [solved, ik_result] = controller->moveit_helper.CalculateIK(state_msg.target_end_effector_pose);
    if (solved) {
      controller->trajectory_tracker.SwitchReferenceTrajectory(ik_result);
      state_msg.last_ik_success = true;
    } else {
      state_msg.last_ik_success = false;
    }

    controller->ik_calculation_mutex.unlock();  // 计算完毕，解锁
  });
  BehaviourTreeExecuting::execution_finished_callback_ = e.execution_finished_callback;
  BehaviourTreeExecuting::execution_interrupted_callback_ = e.execution_interrupted_callback;
  // TODO:有可能出现ik计算还没有完成就transit了的情况，这时候上一个轨迹已经执行完毕的话Executing状态会立刻返回到Idle状态。这个逻辑需要想一想怎么处理，暂时不要用ArmEndEffectorCartesianCommand这个接口
  DispatchEvent(event::SwitchMode<BehaviourTreeExecuting>{});
}
void BehaviourTreeIdle::react(event::BehaviourTreeCommand<fluorite_msgs::action::ArmJointCommand::Goal> const &e) {
  const auto kTargetJointPosition = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint ret;
    ret.velocities.resize(6);
    ret.positions = e.command.goal_joint_state.position;
    return ret;
  }();
  controller->trajectory_tracker.SwitchReferenceTrajectory(kTargetJointPosition);
  BehaviourTreeExecuting::execution_finished_callback_ = e.execution_finished_callback;
  BehaviourTreeExecuting::execution_interrupted_callback_ = e.execution_interrupted_callback;
  DispatchEvent(event::SwitchMode<BehaviourTreeExecuting>{});
}
void BehaviourTreeIdle::react(event::BehaviourTreeCommand<fluorite_msgs::action::ArmTrajectoryCommand::Goal> const &e) {
  controller->trajectory_tracker.SwitchReferenceTrajectory(e.command.trajectory);
  BehaviourTreeExecuting::execution_finished_callback_ = e.execution_finished_callback;
  BehaviourTreeExecuting::execution_interrupted_callback_ = e.execution_interrupted_callback;
  DispatchEvent(event::SwitchMode<BehaviourTreeExecuting>{});
}
void BehaviourTreeExecuting::react(event::Control const &control) {
  if (controller->trajectory_tracker.execution_finished() ||
      controller->hotkeys.force_finish_execution.have_pressed()) {
    if (execution_finished_callback_) {
      execution_finished_callback_();
      execution_finished_callback_ = nullptr;
    }
    DispatchEvent(event::SwitchMode<BehaviourTreeIdle>{});
  }
  Arm::CalcPidAndExecute();

  // state transition
  if (control.sd != kRcSwitchUp) {
    if (execution_interrupted_callback_) {
      execution_interrupted_callback_();
    }
    DispatchEvent(event::SwitchMode<NoForce>{});
    return;
  }
  if (control.sf == kRcSwitchUp) {
    if (execution_interrupted_callback_) {
      execution_interrupted_callback_();
    }
    DispatchEvent(event::SwitchMode<Debug>{});
    return;
  }
  switch (control.sc) {
    case kRcSwitchUp: {
      if (controller->hotkeys.toggle_custom_ctrl.have_pressed()) {
        if (execution_interrupted_callback_) {
          execution_interrupted_callback_();
        }
        DispatchEvent(event::SwitchMode<CustomCtrlManual>{});
      }
      return;
    }
    case kRcSwitchMiddle: {
      if (execution_interrupted_callback_) {
        execution_interrupted_callback_();
      }
      DispatchEvent(event::SwitchMode<KeyboardManual>{});
      return;
    }
    case kRcSwitchDown: {
      if (execution_interrupted_callback_) {
        execution_interrupted_callback_();
      }
      DispatchEvent(event::SwitchMode<Retracted>{});
      return;
    }
  }
}
void BehaviourTreeExecuting::react(event::BehaviorTreeCancelExecution const &e) {
  if (execution_interrupted_callback_) {
    execution_interrupted_callback_();
  }
  DispatchEvent(event::SwitchMode<BehaviourTreeIdle>{});
}
void Retracted::react(event::Control const &control) {
  Arm::CalcPidAndExecute();

  // state transition
  if (control.sd != kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<NoForce>{});
    return;
  }
  if (control.sf == kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<Debug>{});
    return;
  }
  switch (control.sc) {
    case kRcSwitchUp: {
      DispatchEvent(event::SwitchMode<BehaviourTreeIdle>{});
      return;
    }
    case kRcSwitchMiddle: {
      DispatchEvent(event::SwitchMode<KeyboardManual>{});
      return;
    }
    case kRcSwitchDown: {
      return;
    }
  }
}
void NoForce::react(event::Control const &control) {
  Arm::EnableActuators(false);
  actuator->end_yaw_2.SetCurrent(0);

  // state transition
  if (control.sd != kRcSwitchUp) {
    return;
  }
  if (control.sf == kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<Debug>{});
    return;
  }
  switch (control.sc) {
    case kRcSwitchUp: {
      DispatchEvent(event::SwitchMode<KeyboardManual>{});
      return;
    }
    case kRcSwitchMiddle: {
      DispatchEvent(event::SwitchMode<BehaviourTreeIdle>{});
      return;
    }
    case kRcSwitchDown: {
      DispatchEvent(event::SwitchMode<Retracted>{});
      return;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

}  // namespace arm

namespace gimbal_base {

#define UpdateState()                                \
  switch (control.sb) {                              \
    case kRcSwitchUp: {                              \
      DispatchEvent(event::SwitchMode<FollowImu>{}); \
      break;                                         \
    }                                                \
    case kRcSwitchMiddle: {                          \
      DispatchEvent(event::SwitchMode<NoForce>{});   \
      break;                                         \
    }                                                \
    case kRcSwitchDown: {                            \
      DispatchEvent(event::SwitchMode<NoForce>{});   \
      break;                                         \
    }                                                \
  }

void FollowImu::entry() {
  state_msg.imu_yaw_target =
      rm::modules::algorithm::utils::LoopConstrain(state_msg.imu_yaw - state_msg.yaw_encoder, -M_PIf, M_PIf);
  actuator->base_yaw.Enable(true);
}
void NoForce::entry() { actuator->base_yaw.Enable(false); }

void FollowImu::react(event::Control const &control) {
  const double keyboard_q_e = 0.003 * (static_cast<int>(control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_E)) -
                                       static_cast<int>(control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_Q)));
  state_msg.imu_yaw_target = rm::modules::algorithm::utils::LoopConstrain(
      state_msg.imu_yaw_target - keyboard_q_e - control.right_x * 0.006 - control.mouse_x, -M_PIf, M_PIf);

  const auto base_yaw_imu =
      rm::modules::algorithm::utils::LoopConstrain(state_msg.imu_yaw - state_msg.yaw_encoder, -M_PIf, M_PIf);
  controller->base_yaw_position_pid.Update(state_msg.imu_yaw_target, base_yaw_imu);
  controller->base_yaw_speed_pid.Update(controller->base_yaw_position_pid.value(), state_msg.base_yaw_encoder_speed);
  actuator->base_yaw.Set(controller->base_yaw_speed_pid.value());
  UpdateState();
}
void NoForce::react(event::Control const &control) {
  actuator->base_yaw.Enable(false);
  controller->base_yaw_position_pid.Clear();
  controller->base_yaw_speed_pid.Clear();
  UpdateState();
}

#undef UpdateState

}  // namespace gimbal_base

namespace pneumatic_system {

#define UpdateState()                               \
  switch (control.sd) {                             \
    case kRcSwitchUp: {                             \
      if (control.sf == kRcSwitchUp) {              \
        DispatchEvent(event::SwitchMode<Debug>{});  \
      } else {                                      \
        DispatchEvent(event::SwitchMode<Normal>{}); \
      }                                             \
      break;                                        \
    }                                               \
    case kRcSwitchMiddle: {                         \
      DispatchEvent(event::SwitchMode<Off>{});      \
      break;                                        \
    }                                               \
    case kRcSwitchDown: {                           \
      DispatchEvent(event::SwitchMode<Off>{});      \
      break;                                        \
    }                                               \
  }

void Normal::entry() {
  actuator->sucker_system.Set(SuckerSystem::SuckerState::kOff);
  actuator->storage_system.SetLeft(false);
  actuator->storage_system.SetRight(false);
}
void Debug::entry() {}
void Off::entry() {
  actuator->sucker_system.Set(SuckerSystem::SuckerState::kOff);
  actuator->storage_system.SetLeft(false);
  actuator->storage_system.SetRight(false);
}

void Normal::react(event::Control const &control) {
  if (controller->hotkeys.toggle_arm_pump.have_pressed()) {
    actuator->sucker_system.CycleMode();
  }
  if (controller->hotkeys.toggle_left_storage.have_pressed()) {
    actuator->storage_system.ToggleLeft();
  }
  if (controller->hotkeys.toggle_right_storage.have_pressed()) {
    actuator->storage_system.ToggleRight();
  }
  UpdateState();
}
void Normal::react(event::BehaviourTreeCommand<fluorite_msgs::srv::StorageSystemCommand::Request> const &e) {
  switch (e.command.left) {
    case fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_DONT_CHANGE: {
      break;
    }
    case fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_ON: {
      actuator->storage_system.SetLeft(true);
      break;
    }
    case fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_OFF: {
      actuator->storage_system.SetLeft(false);
      break;
    }
  }
  switch (e.command.right) {
    case fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_DONT_CHANGE: {
      break;
    }
    case fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_ON: {
      actuator->storage_system.SetRight(true);
      break;
    }
    case fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_OFF: {
      actuator->storage_system.SetRight(false);
      break;
    }
  }
}
void Normal::react(event::BehaviourTreeCommand<fluorite_msgs::srv::SuckerCommand::Request> const &e) {
  switch (e.command.command) {
    case fluorite_msgs::srv::SuckerCommand::Request::SUCKER_OFF: {
      actuator->sucker_system.Set(SuckerSystem::SuckerState::kOff);
      break;
    }
    case fluorite_msgs::srv::SuckerCommand::Request::SUCKER_SUCK: {
      actuator->sucker_system.Set(SuckerSystem::SuckerState::kSuck);
      break;
    }
    case fluorite_msgs::srv::SuckerCommand::Request::SUCKER_BLOW: {
      actuator->sucker_system.Set(SuckerSystem::SuckerState::kBlow);
      break;
    }
  }
}
void Debug::react(event::Control const &control) {
  if (control.ld > 0.5) {
    actuator->sucker_system.Set(SuckerSystem::SuckerState::kSuck);
  } else if (control.ld < -0.5) {
    actuator->sucker_system.Set(SuckerSystem::SuckerState::kBlow);
  } else {
    actuator->sucker_system.Set(SuckerSystem::SuckerState::kOff);
  }
  actuator->storage_system.SetLeft(control.ls > 0);
  actuator->storage_system.SetRight(control.rs > 0);
  UpdateState();
}
void Off::react(event::Control const &control) { UpdateState(); }

#undef UpdateState

}  // namespace pneumatic_system

}  // namespace state

}  // namespace fsm

}  // namespace fluorite::arm

FSM_INITIAL_STATE(fluorite::arm::fsm::Arm, fluorite::arm::fsm::state::arm::NoForce);
FSM_INITIAL_STATE(fluorite::arm::fsm::GimbalBase, fluorite::arm::fsm::state::gimbal_base::NoForce);
FSM_INITIAL_STATE(fluorite::arm::fsm::PneumaticSystem, fluorite::arm::fsm::state::pneumatic_system::Off);
