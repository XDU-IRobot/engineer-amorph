/**
 * @file  arm/fsm.cc
 * @brief 状态机实现
 */

#include "arm/fsm.hpp"

#include <iostream>
#include <unordered_map>

#include <Eigen/Dense>

#include "keyboard_msgs/msg/key.hpp"
#include "tinyfsm.hpp"

constexpr int kRcSwitchUp = 1;
constexpr int kRcSwitchMiddle = 0;
constexpr int kRcSwitchDown = -1;

namespace fluorite {

fluorite_msgs::msg::ArmState state{};
std::unique_ptr<Actuator> actuator{nullptr};
std::unique_ptr<Controller> controller{nullptr};

namespace fsm {

// 忽略不关心的event
// these functions should never be called so wtf in them
void Arm::react(tinyfsm::Event const &event) { error("wtf1"); }
void Arm::react(event::Control const &control) { error("wt2f"); }
void GimbalBase::react(tinyfsm::Event const &event) { error("wtf3"); }
void GimbalBase::react(event::Control const &control) { error("wt4f"); }

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

  // apply output torque
  const auto &kOutputTorque = fluorite::controller->trajectory_tracker.output_torque();
  fluorite::actuator->yaw.SetPosition(0, 0, kOutputTorque[0], 0, 0);
  fluorite::actuator->pitch_1.SetPosition(0, 0, kOutputTorque[1], 0, 0);
  fluorite::actuator->pitch_2.SetPosition(0, 0, kOutputTorque[2], 0, 0);
  fluorite::actuator->end_yaw_1.SetPosition(0, 0, kOutputTorque[3], 0, 0);
  fluorite::actuator->end_pitch.SetPosition(0, 0, kOutputTorque[4], 0, 0);
  fluorite::actuator->end_yaw_2.SetCurrent(kOutputTorque[5]);
}

namespace state {

namespace arm {

void Debug::entry() {
  Arm::EnableActuators(true);
  const auto kCurrentJointPositions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.resize(6);
    p.velocities.resize(6);
    p.positions[0] = fluorite::state.yaw_encoder;
    p.positions[1] = fluorite::state.pitch_1_encoder;
    p.positions[2] = fluorite::state.pitch_2_encoder;
    p.positions[3] = fluorite::state.end_yaw_1_encoder;
    p.positions[4] = fluorite::state.end_pitch_encoder;
    p.positions[5] = fluorite::state.end_yaw_2_encoder;
    return p;
  }();
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kCurrentJointPositions);
}
void KeyboardManual::entry() {
  Arm::EnableActuators(true);
  // 查询tf树获取当前末端位姿
  try {
    const auto t =
        fluorite::controller->tf_buffer->lookupTransform("base_yaw_link", "end_yaw_2_link", tf2::TimePointZero);
    fluorite::state.target_end_effector_pose.position.x = t.transform.translation.x;
    fluorite::state.target_end_effector_pose.position.y = t.transform.translation.y;
    fluorite::state.target_end_effector_pose.position.z = t.transform.translation.z;
    fluorite::state.target_end_effector_pose.orientation.x = t.transform.rotation.x;
    fluorite::state.target_end_effector_pose.orientation.y = t.transform.rotation.y;
    fluorite::state.target_end_effector_pose.orientation.z = t.transform.rotation.z;
    fluorite::state.target_end_effector_pose.orientation.w = t.transform.rotation.w;
  } catch (tf2::TransformException &ex) {
    error("%s", ex.what());
  }
  const auto kCurrentJointPositions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.resize(6);
    p.velocities.resize(6);
    p.positions[0] = fluorite::state.yaw_encoder;
    p.positions[1] = fluorite::state.pitch_1_encoder;
    p.positions[2] = fluorite::state.pitch_2_encoder;
    p.positions[3] = fluorite::state.end_yaw_1_encoder;
    p.positions[4] = fluorite::state.end_pitch_encoder;
    p.positions[5] = fluorite::state.end_yaw_2_encoder;
    return p;
  }();
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kCurrentJointPositions);
}
void CustomCtrlManual::entry() {
  Arm::EnableActuators(true);
  // 查询tf树获取当前末端位姿
  try {
    const auto t =
        fluorite::controller->tf_buffer->lookupTransform("base_yaw_link", "end_yaw_2_link", tf2::TimePointZero);
    fluorite::state.target_end_effector_pose.position.x = t.transform.translation.x;
    fluorite::state.target_end_effector_pose.position.y = t.transform.translation.y;
    fluorite::state.target_end_effector_pose.position.z = t.transform.translation.z;
    fluorite::state.target_end_effector_pose.orientation.x = t.transform.rotation.x;
    fluorite::state.target_end_effector_pose.orientation.y = t.transform.rotation.y;
    fluorite::state.target_end_effector_pose.orientation.z = t.transform.rotation.z;
    fluorite::state.target_end_effector_pose.orientation.w = t.transform.rotation.w;
  } catch (tf2::TransformException &ex) {
    error("%s", ex.what());
  }
  const auto kCurrentJointPositions = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.resize(6);
    p.velocities.resize(6);
    p.positions[0] = fluorite::state.yaw_encoder;
    p.positions[1] = fluorite::state.pitch_1_encoder;
    p.positions[2] = fluorite::state.pitch_2_encoder;
    p.positions[3] = fluorite::state.end_yaw_1_encoder;
    p.positions[4] = fluorite::state.end_pitch_encoder;
    p.positions[5] = fluorite::state.end_yaw_2_encoder;
    return p;
  }();
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kCurrentJointPositions);
}
void FollowExternalTrajectory::entry() { Arm::EnableActuators(true); }
void Retracted::entry() {
  Arm::EnableActuators(true);
  fluorite::controller->trajectory_tracker.ClearPid();
}
void NoForce::entry() {
  Arm::EnableActuators(false);
  fluorite::controller->trajectory_tracker.ClearPid();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////CONTROL LOOPS///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// 根据遥控器拨杆和键盘的状态，更新机械臂的运行模式
// 转换逻辑请看仓库最顶层的README.md，操作方法一章
// 每次控制循环运行结束后，都会调用这个宏一次
#define UpdateState()                                                           \
  do {                                                                          \
    if (control.sd != kRcSwitchUp) {                                            \
      DispatchEvent(event::SwitchMode<NoForce>{});                              \
      return;                                                                   \
    }                                                                           \
    if (control.sf == kRcSwitchUp) {                                            \
      DispatchEvent(event::SwitchMode<Debug>{});                                \
      return;                                                                   \
    }                                                                           \
    switch (control.sc) {                                                       \
      case kRcSwitchUp: {                                                       \
        if (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_G) == false) { \
          DispatchEvent(event::SwitchMode<KeyboardManual>{});                   \
        } else {                                                                \
          DispatchEvent(event::SwitchMode<CustomCtrlManual>{});                 \
        }                                                                       \
        return;                                                                 \
      }                                                                         \
      case kRcSwitchMiddle: {                                                   \
        DispatchEvent(event::SwitchMode<FollowExternalTrajectory>{});           \
        return;                                                                 \
      }                                                                         \
      case kRcSwitchDown: {                                                     \
        DispatchEvent(event::SwitchMode<Retracted>{});                          \
        return;                                                                 \
      }                                                                         \
    }                                                                           \
  } while (0)

void Debug::react(event::Control const &control) {
  auto joint_positions = fluorite::controller->trajectory_tracker.current_point();
  if (control.se == kRcSwitchUp) {
    joint_positions.positions[0] += control.left_y * 0.0017;
  } else if (control.se == kRcSwitchMiddle) {
    joint_positions.positions[1] += control.left_y * 0.0017;
  } else {
    if (control.sg == kRcSwitchUp) {
      joint_positions.positions[2] += control.left_y * 0.0017;
    } else if (control.sg == kRcSwitchMiddle) {
      joint_positions.positions[3] += control.left_y * 0.0017;
    } else {
      if (control.sh == kRcSwitchUp) {
        joint_positions.positions[4] += control.left_y * 0.0017;
      } else if (control.sh == kRcSwitchDown) {
        joint_positions.positions[5] += control.left_y * 0.0017;
      }
    }
  }
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(joint_positions);
  Arm::CalcPidAndExecute();
  UpdateState();
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
        try {
          // 尝试逆解，成功的话就把解出来的关节位置送进轨迹跟踪器
          const auto joint_model_group = fluorite::controller->kinematic_model->getJointModelGroup("fluorite_arm");
          const bool ik_solved = fluorite::controller->kinematic_model->setFromIK(
              joint_model_group, fluorite::state.target_end_effector_pose, 0.1);
          fluorite::controller->kinematic_model->enforceBounds();
          if (ik_solved && fluorite::controller->kinematic_model->satisfiesBounds()) {
            std::vector<double> joint_values;
            fluorite::state.last_ik_success = true;
            fluorite::controller->kinematic_model->copyJointGroupPositions(joint_model_group, joint_values);
            info("====================================================");
            info("ik solved!");
            for (std::size_t i = 0; i < joint_values.size(); ++i) {
              info("Joint %lu: %f", i, joint_values[i]);
            }
            info("====================================================");
            const auto kReferenceTraj = [&] {
              trajectory_msgs::msg::JointTrajectoryPoint p;
              p.positions = joint_values;
              p.velocities.resize(6);
              return p;
            }();
            fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kReferenceTraj);
          } else {
            fluorite::state.last_ik_success = false;
            warn("failed to solve ik");
          }
        } catch (const std::exception &e) {
          error("failed to solve ik: %s", e.what());
          fluorite::controller->ik_calculation_mutex.unlock();
        }

        // 计算完毕，解锁
        fluorite::controller->ik_calculation_mutex.unlock();
      });
    }
  }

  Arm::CalcPidAndExecute();
  UpdateState();
}
void CustomCtrlManual::react(event::Control const &control) {
  // TODO
}
void FollowExternalTrajectory::react(event::Control const &control) {
  // Arm::CalcPidAndExecute();

  UpdateState();
}
void Retracted::react(event::Control const &control) {
  const auto kReferenceTraj = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {0.0, 0.1, 0.1, 0, 0, 0};
    p.velocities = {0, 0, 0, 0, 0, 0};
    return p;
  }();

  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kReferenceTraj);
  Arm::CalcPidAndExecute();

  UpdateState();
}
void NoForce::react(event::Control const &control) {
  Arm::EnableActuators(false);
  fluorite::actuator->end_yaw_2.SetCurrent(0);
  UpdateState();
}

#undef UpdateState

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

#define UpdateState()                          \
  switch (control.sd) {                        \
    case kRcSwitchUp: {                        \
      DispatchEvent(event::SwitchMode<On>{});  \
      break;                                   \
    }                                          \
    case kRcSwitchMiddle: {                    \
      DispatchEvent(event::SwitchMode<Off>{}); \
      break;                                   \
    }                                          \
    case kRcSwitchDown: {                      \
      DispatchEvent(event::SwitchMode<Off>{}); \
      break;                                   \
    }                                          \
  }

void On::entry() {}
void Off::entry() {}

void On::react(event::Control const &control) {
  fluorite::actuator->sucker_system.Suck(control.ld);
  fluorite::actuator->storage_system.SetPumpPower(control.rd);
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