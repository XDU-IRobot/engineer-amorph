/**
 * @file  arm/fsm.cc
 * @brief 状态机实现
 */

#include "arm/fsm.hpp"

#include <chrono>
#include <iostream>
#include <unordered_map>

#include <Eigen/Dense>

#include "keyboard_msgs/msg/key.hpp"
#include "tinyfsm.hpp"

namespace fluorite {

fluorite_msgs::msg::ArmState state{};
std::unique_ptr<Actuator> actuator{nullptr};
std::unique_ptr<Controller> controller{nullptr};

namespace fsm {

// 忽略不关心的event
// these functions should never be called so wtf in them
void Arm::react(tinyfsm::Event const &event) { error("FSM: Arm received unknown event"); }
void GimbalBase::react(tinyfsm::Event const &event) { error("FSM: GimbalBase received unknown event"); }
void PneumaticSystem::react(tinyfsm::Event const &event) { error("FSM: PneumaticSystem received unknown event"); }

void Arm::EnableActuators(bool enable) {
  const auto dm_enable = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
  fluorite::actuator->yaw.SendInstruction(dm_enable);
  fluorite::actuator->pitch_1.SendInstruction(dm_enable);
  fluorite::actuator->pitch_2.SendInstruction(dm_enable);
  fluorite::actuator->end_yaw_1.SendInstruction(dm_enable);
  fluorite::actuator->end_pitch.SendInstruction(dm_enable);
}

void Arm::CalcPidAndExecute() {
  const std::array<double, 6> kCurrentPosition = {fluorite::state.yaw_encoder,       fluorite::state.pitch_1_encoder,
                                                  fluorite::state.pitch_2_encoder,   fluorite::state.end_yaw_1_encoder,
                                                  fluorite::state.end_pitch_encoder, fluorite::state.end_yaw_2_encoder};
  const std::array<double, 6> kCurrentSpeed = {
      fluorite::state.yaw_encoder_speed,       fluorite::state.pitch_1_encoder_speed,
      fluorite::state.pitch_2_encoder_speed,   fluorite::state.end_yaw_1_encoder_speed,
      fluorite::state.end_pitch_encoder_speed, fluorite::state.end_yaw_2_encoder_speed};

  fluorite::controller->trajectory_tracker.Update(kCurrentPosition, kCurrentSpeed);

  // 重力补偿
  const auto kCompensateTorques = fluorite::controller->dynamic_model(
      {fluorite::state.yaw_encoder, fluorite::state.pitch_1_encoder, fluorite::state.pitch_2_encoder},
      {fluorite::state.yaw_encoder_speed, fluorite::state.pitch_1_encoder_speed, fluorite::state.pitch_2_encoder_speed},
      {0.0, 0.0, 0.0}, false);

  // apply output torque
  const auto kOutputTorque = fluorite::controller->trajectory_tracker.output_torque();
  const auto kDesiredPosition = fluorite::controller->trajectory_tracker.current_point().positions;
  fluorite::actuator->yaw.SetPosition(kDesiredPosition[0], 0, 0, 90, 35);
  fluorite::actuator->pitch_1.SetPosition(kDesiredPosition[1] * 1.5,  // 传动链条 减速比2:3
                                          0, 0.65 * kCompensateTorques[1], 85, 35);
  fluorite::actuator->pitch_2.SetPosition(kDesiredPosition[2], 0, 0.65 * kCompensateTorques[2], 70, 30);
  fluorite::actuator->end_yaw_1.SetPosition(kDesiredPosition[3], 0, 0, 70, 30);
  fluorite::actuator->end_pitch.SetPosition(kDesiredPosition[4], 0, 0, 70, 30);
  fluorite::actuator->end_yaw_2.SetCurrent(kOutputTorque[5]);
}

namespace state {

namespace arm {

void Debug::entry() {
  Arm::EnableActuators(true);
  const auto kCurrentJointPositions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.velocities.resize(6);
    p.positions = {fluorite::state.yaw_encoder,       fluorite::state.pitch_1_encoder,
                   fluorite::state.pitch_2_encoder,   fluorite::state.end_yaw_1_encoder,
                   fluorite::state.end_pitch_encoder, fluorite::state.end_yaw_2_encoder};
    return p;
  }();
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kCurrentJointPositions);
}
void KeyboardManual::entry() {
  Arm::EnableActuators(true);
  const auto kCurrentJointPositions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.resize(6);
    p.velocities.resize(6);
    p.positions = {fluorite::state.yaw_encoder,       fluorite::state.pitch_1_encoder,
                   fluorite::state.pitch_2_encoder,   fluorite::state.end_yaw_1_encoder,
                   fluorite::state.end_pitch_encoder, fluorite::state.end_yaw_2_encoder};
    return p;
  }();
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kCurrentJointPositions);
}
void CustomCtrlManual::entry() {
  Arm::EnableActuators(true);
  const auto kCurrentJointPositions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.velocities.resize(6);
    p.positions = {fluorite::state.yaw_encoder,       fluorite::state.pitch_1_encoder,
                   fluorite::state.pitch_2_encoder,   fluorite::state.end_yaw_1_encoder,
                   fluorite::state.end_pitch_encoder, fluorite::state.end_yaw_2_encoder};
    return p;
  }();
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kCurrentJointPositions);
}
void BehaviourTreeIdle::entry() {
  Arm::EnableActuators(true);
  const auto kCurrentJointPositions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.velocities.resize(6);
    p.positions = {fluorite::state.yaw_encoder,       fluorite::state.pitch_1_encoder,
                   fluorite::state.pitch_2_encoder,   fluorite::state.end_yaw_1_encoder,
                   fluorite::state.end_pitch_encoder, fluorite::state.end_yaw_2_encoder};
    return p;
  }();
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kCurrentJointPositions);
}
std::function<void()> BehaviourTreeExecuting::execution_finished_callback_;
std::function<void()> BehaviourTreeExecuting::execution_interrupted_callback_;
void BehaviourTreeExecuting::entry() { Arm::EnableActuators(true); }
void Retracted::entry() {
  Arm::EnableActuators(true);
  const auto retracted_joint_positions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {0.0, 0.1, 0.1, 0, 0, 0};
    p.velocities = {0, 0, 0, 0, 0, 0};
    return p;
  }();

  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(retracted_joint_positions,
                                                                     std::chrono::seconds(1));
}
void NoForce::entry() {
  Arm::EnableActuators(false);
  fluorite::controller->trajectory_tracker.ClearPid();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////CONTROL LOOPS///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Debug::react(event::Control const &control) {
  auto joint_positions = fluorite::controller->trajectory_tracker.current_point();
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
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(joint_positions);
  Arm::CalcPidAndExecute();

  // update hotkey
  controller->hotkeys.toggle_custom_ctrl.Poll(control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_G));

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
        fluorite::state.target_end_effector_pose.orientation.w, fluorite::state.target_end_effector_pose.orientation.x,
        fluorite::state.target_end_effector_pose.orientation.y, fluorite::state.target_end_effector_pose.orientation.z);
    const Eigen::AngleAxisd yaw_angle(delta_yaw, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd pitch_angle(delta_pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd roll_angle(delta_roll, Eigen::Vector3d::UnitX());
    q = q * yaw_angle * pitch_angle * roll_angle;

    fluorite::state.target_end_effector_pose.position.x += delta_x;
    fluorite::state.target_end_effector_pose.position.y += delta_y;
    fluorite::state.target_end_effector_pose.position.z += delta_z;
    fluorite::state.target_end_effector_pose.orientation.x = q.x();
    fluorite::state.target_end_effector_pose.orientation.y = q.y();
    fluorite::state.target_end_effector_pose.orientation.z = q.z();
    fluorite::state.target_end_effector_pose.orientation.w = q.w();

    // 如果上一轮计算结束（锁被释放），才计算一次新的目标关节位置，否则还是按TrajectoryTracker中的参考轨迹继续运动
    if (fluorite::controller->ik_calculation_mutex.try_lock()) {
      fluorite::controller->ik_calculation_thread_pool_.enqueue([&] {  // 异步计算ik，不阻塞控制循环
        const auto [kSolved, kIKResult] =
            fluorite::controller->moveit_helper.CalculateIK(fluorite::state.target_end_effector_pose);
        if (kSolved) {
          fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kIKResult);
          fluorite::state.last_ik_success = true;
        } else {
          fluorite::state.last_ik_success = false;
        }

        fluorite::controller->ik_calculation_mutex.unlock();  // 计算完毕，解锁
      });
    }
  }

  Arm::CalcPidAndExecute();

  // update hotkey
  controller->hotkeys.toggle_custom_ctrl.Poll(control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_G));

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
      fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(target_joint_positions);
    }
  } else if constexpr (std::is_same_v<decltype(control.custom_controller_command),
                                      custom_controller::CartesianCommand>) {
    // TODO: 感觉T265自定义控制器不会再用了
  }
  Arm::CalcPidAndExecute();

  // update hotkey
  controller->hotkeys.toggle_custom_ctrl.Poll(control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_G));

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
  Arm::CalcPidAndExecute();

  // update hotkey
  controller->hotkeys.toggle_custom_ctrl.Poll(control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_G));

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
  fluorite::controller->ik_calculation_thread_pool_.enqueue([&] {  // 异步计算ik，不阻塞控制循环
    fluorite::controller->ik_calculation_mutex.lock();             // 阻塞等待上一次ik计算完成
    // 尝试逆解，成功的话就把解出来的关节位置送进轨迹跟踪器
    fluorite::state.target_end_effector_pose = e.command.goal_end_effector_pose;
    const auto [solved, ik_result] =
        fluorite::controller->moveit_helper.CalculateIK(fluorite::state.target_end_effector_pose);
    if (solved) {
      fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(ik_result);
      fluorite::state.last_ik_success = true;
    } else {
      fluorite::state.last_ik_success = false;
    }

    fluorite::controller->ik_calculation_mutex.unlock();  // 计算完毕，解锁
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
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kTargetJointPosition);
  BehaviourTreeExecuting::execution_finished_callback_ = e.execution_finished_callback;
  BehaviourTreeExecuting::execution_interrupted_callback_ = e.execution_interrupted_callback;
  DispatchEvent(event::SwitchMode<BehaviourTreeExecuting>{});
}
void BehaviourTreeIdle::react(event::BehaviourTreeCommand<fluorite_msgs::action::ArmTrajectoryCommand::Goal> const &e) {
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(e.command.trajectory);
  BehaviourTreeExecuting::execution_finished_callback_ = e.execution_finished_callback;
  BehaviourTreeExecuting::execution_interrupted_callback_ = e.execution_interrupted_callback;
  DispatchEvent(event::SwitchMode<BehaviourTreeExecuting>{});
}
void BehaviourTreeExecuting::react(event::Control const &control) {
  if (fluorite::controller->trajectory_tracker.execution_finished()) {
    if (execution_finished_callback_) {
      execution_finished_callback_();
    }
    DispatchEvent(event::SwitchMode<BehaviourTreeIdle>{});
  }
  Arm::CalcPidAndExecute();

  // update hotkey
  controller->hotkeys.toggle_custom_ctrl.Poll(control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_G));

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

  // update hotkey
  controller->hotkeys.toggle_custom_ctrl.Poll(control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_G));

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
  fluorite::actuator->end_yaw_2.SetCurrent(0);

  // update hotkey
  controller->hotkeys.toggle_custom_ctrl.Poll(control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_G));

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
  fluorite::state.imu_yaw_target = rm::modules::algorithm::utils::LoopConstrain(
      fluorite::state.imu_yaw - fluorite::state.yaw_encoder, -M_PIf, M_PIf);
  fluorite::actuator->base_yaw.Enable(true);
}
void NoForce::entry() { fluorite::actuator->base_yaw.Enable(false); }

void FollowImu::react(event::Control const &control) {
  fluorite::state.imu_yaw_target = rm::modules::algorithm::utils::LoopConstrain(
      fluorite::state.imu_yaw_target - control.right_x * 0.006 - control.mouse_x, -M_PIf, M_PIf);

  const auto base_yaw_imu = rm::modules::algorithm::utils::LoopConstrain(
      fluorite::state.imu_yaw - fluorite::state.yaw_encoder, -M_PIf, M_PIf);
  fluorite::controller->base_yaw_position_pid.Update(fluorite::state.imu_yaw_target, base_yaw_imu);
  fluorite::controller->base_yaw_speed_pid.Update(fluorite::controller->base_yaw_position_pid.value(),
                                                  fluorite::state.base_yaw_encoder_speed);
  fluorite::actuator->base_yaw.Set(fluorite::controller->base_yaw_speed_pid.value());
  UpdateState();
}
void NoForce::react(event::Control const &control) {
  fluorite::actuator->base_yaw.Enable(false);
  fluorite::controller->base_yaw_position_pid.Clear();
  fluorite::controller->base_yaw_speed_pid.Clear();
  UpdateState();
}

#undef UpdateState

}  // namespace gimbal_base

namespace pneumatic_system {

#define UpdateState()                             \
  switch (control.sd) {                           \
    case kRcSwitchUp: {                           \
      DispatchEvent(event::SwitchMode<Manual>{}); \
      break;                                      \
    }                                             \
    case kRcSwitchMiddle: {                       \
      DispatchEvent(event::SwitchMode<Off>{});    \
      break;                                      \
    }                                             \
    case kRcSwitchDown: {                         \
      DispatchEvent(event::SwitchMode<Off>{});    \
      break;                                      \
    }                                             \
  }

void Manual::entry() {}
void Off::entry() {}

void Manual::react(event::Control const &control) {
  fluorite::actuator->sucker_system.Suck(control.ld);
  fluorite::actuator->storage_system.SetPumpPower();
  fluorite::actuator->storage_system.SetLeftValve(control.ls > 0);
  fluorite::actuator->storage_system.SetRightValve(control.rs > 0);
  UpdateState();
}

void Off::react(event::Control const &control) {
  fluorite::actuator->sucker_system.Disable();
  fluorite::actuator->storage_system.Disable();
  UpdateState();
}

#undef UpdateState

}  // namespace pneumatic_system

}  // namespace state

}  // namespace fsm

}  // namespace fluorite

FSM_INITIAL_STATE(fluorite::fsm::Arm, fluorite::fsm::state::arm::NoForce);
FSM_INITIAL_STATE(fluorite::fsm::GimbalBase, fluorite::fsm::state::gimbal_base::NoForce);
FSM_INITIAL_STATE(fluorite::fsm::PneumaticSystem, fluorite::fsm::state::pneumatic_system::Off);
