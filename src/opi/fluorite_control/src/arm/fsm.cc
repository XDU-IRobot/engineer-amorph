/**
 * @file  fsm.cc
 * @brief 状态机实现
 */

#include "arm/fsm.hpp"

#include <iostream>
#include <unordered_map>

#include <Eigen/Dense>

#include "keyboard_msgs/msg/key.hpp"

#include "librm.hpp"

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
  fluorite::actuator->base_yaw.Enable(enable);
  const auto dm_enable = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
  fluorite::actuator->yaw.SendInstruction(dm_enable);
  fluorite::actuator->pitch_1.SendInstruction(dm_enable);
  fluorite::actuator->pitch_2.SendInstruction(dm_enable);
  fluorite::actuator->end_yaw_1.SendInstruction(dm_enable);
  fluorite::actuator->end_pitch.SendInstruction(dm_enable);
}

namespace state {

namespace arm {

void Debug::entry() { Arm::EnableActuators(true); }
void KeyboardManual::entry() { Arm::EnableActuators(true); }
void CustomCtrlManual::entry() { Arm::EnableActuators(true); }
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

void Debug::react(event::Control const &control) {}
ThreadPool KeyboardManual::move_group_request_thread_pool_{1};
void KeyboardManual::react(event::Control const &control) {
  // 如果键盘有输入，
  if ((control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_D) ||
       control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_A) ||
       control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_S) ||
       control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_W) ||
       control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_R) ||
       control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_F) ||
       control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_U) ||
       control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_O))) {
    //并且上一轮规划结束（锁被释放），才进行一轮规划，否则只是按旧target执行一轮控制
    if (fluorite::controller->move_group_request_mutex.try_lock()) {
      move_group_request_thread_pool_.enqueue([&] {  // 异步执行规划，防止阻塞主控制循环
        try {
          const auto target_pose = [&] {  // 根据键盘输入计算末端的目标位姿
            auto ret = fluorite::controller->move_group.getCurrentPose();

            const double delta_x = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_D) -
                                   control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_A) * 0.1;
            const double delta_y = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_S) -
                                   control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_W) * 0.1;
            const double delta_z = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_R) -
                                   control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_F) * 0.1;
            const double delta_yaw = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_U) -
                                     control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_O) * 0.1;
            double delta_pitch{0}, delta_roll{0};
            if (control.keyboard_modifier & keyboard_msgs::msg::Key::MODIFIER_LSHIFT) {
              delta_pitch = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_I) -
                            control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_K) * 0.02;
              delta_roll = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_J) -
                           control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_L) * 0.02;
            }

            const Eigen::Quaterniond q(fluorite::state.end_effector_pose.transform.rotation.w,
                                       fluorite::state.end_effector_pose.transform.rotation.x,
                                       fluorite::state.end_effector_pose.transform.rotation.y,
                                       fluorite::state.end_effector_pose.transform.rotation.z);
            Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
            euler.x() += delta_roll * 0.01;
            euler.y() += delta_pitch * 0.01;
            euler.z() += delta_yaw * 0.01;

            ret.pose.position.x += delta_x;
            ret.pose.position.y += delta_y;
            ret.pose.position.z += delta_z;

            return ret;
          }();

          // 向move_group提交规划请求
          fluorite::controller->move_group.setNumPlanningAttempts(10);
          fluorite::controller->move_group.setPlanningTime(0.5);
          fluorite::controller->move_group.setPoseTarget(target_pose);
          const auto [success, plan] = [&] {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            return std::make_pair(fluorite::controller->move_group.plan(plan), plan);
          }();

          // 如果规划成功，更新规划结果到PID轨迹追踪器中
          if (success) {
            fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(plan.trajectory_.joint_trajectory);
          } else {
            warn("failed to plan trajectory: invalid target position");
          }
        } catch (const std::exception &e) {
          error("failed to plan trajectory: %s", e.what());
          fluorite::controller->move_group_request_mutex.unlock();
        }

        // 解锁move_group
        fluorite::controller->move_group_request_mutex.unlock();
      });
    }
  }

  const std::array<double, 6> kCurrentPosition = {fluorite::state.imu_yaw,           fluorite::state.pitch_1_encoder,
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

  if (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_G) == true &&
      !(control.keyboard_modifier & keyboard_msgs::msg::Key::MODIFIER_LSHIFT) &&
      !(control.keyboard_modifier & keyboard_msgs::msg::Key::MODIFIER_LCTRL) &&
      !(control.keyboard_modifier & keyboard_msgs::msg::Key::MODIFIER_LALT)) {
    // equivalent to transit<CustomCtrlManual>(), but with logging
    DispatchEvent(event::SwitchMode<CustomCtrlManual>{});
  }
}
void CustomCtrlManual::react(event::Control const &control) {
  // TODO
  if (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_G) == false) {
    // equivalent to transit<CustomCtrlManual>(), but with logging
    DispatchEvent(event::SwitchMode<KeyboardManual>{});
  }
}
void FollowExternalTrajectory::react(event::Control const &control) {
  const std::array<double, 6> kCurrentPosition = {fluorite::state.imu_yaw,           fluorite::state.pitch_1_encoder,
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
void Retracted::react(event::Control const &control) {
  fluorite::state.imu_yaw_target += control.left_x;
  const auto kReferenceTraj = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {fluorite::state.imu_yaw_target, 0, 0, 0, 0, 0};
    p.velocities = {0, 0, 0, 0, 0, 0};
    return p;
  }();
  const std::array<double, 6> kCurrentPosition = {fluorite::state.imu_yaw,           fluorite::state.pitch_1_encoder,
                                                  fluorite::state.pitch_2_encoder,   fluorite::state.end_yaw_1_encoder,
                                                  fluorite::state.end_pitch_encoder, fluorite::state.end_yaw_2_encoder};
  const std::array<double, 6> kCurrentSpeed = {
      fluorite::state.yaw_encoder_speed,       fluorite::state.pitch_1_encoder_speed,
      fluorite::state.pitch_2_encoder_speed,   fluorite::state.end_yaw_1_encoder_speed,
      fluorite::state.end_pitch_encoder_speed, fluorite::state.end_yaw_2_encoder_speed};

  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kReferenceTraj);
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
void NoForce::react(event::Control const &control) {
  Arm::EnableActuators(false);
  fluorite::actuator->end_yaw_2.SetCurrent(0);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

}  // namespace arm

namespace gimbal_base {

void FollowArm::entry() { fluorite::actuator->base_yaw.Enable(true); }
void FollowArmLazy::entry() {
  lazy_state_ = LazyState::kStay;
  fluorite::actuator->base_yaw.Enable(true);
}
void NoForce::entry() {}

void FollowArm::react(event::Control const &control) {
  fluorite::controller->base_yaw_position_pid.Update(0, fluorite::state.yaw_encoder);
  fluorite::controller->base_yaw_speed_pid.Update(0, fluorite::state.base_yaw_encoder_speed);
  fluorite::actuator->base_yaw.Set(fluorite::controller->base_yaw_position_pid.value());
}
double FollowArmLazy::lazily_stay_at_ = 0;
FollowArmLazy::LazyState FollowArmLazy::lazy_state_ = LazyState::kStay;
void FollowArmLazy::react(event::Control const &control) {
  // 如果yaw轴编码器超过60度(1.0472rad)，就尝试把error控制到50度(0.8727rad)，误差小于55度(0.9599rad)之后大yaw轴进入lazy状态，大yaw轴停留在当前位置，直到小yaw轴编码器再次超过60度，lazy状态解除，再次把error控制到50度
  // 体现出来的效果就是，大yaw轴只有在小yaw轴快碰到转动限位的时候才会跟随小yaw轴转动
  switch (lazy_state_) {
    case LazyState::kStay: {
      fluorite::controller->base_yaw_position_pid.Update(lazily_stay_at_,
                                                         fluorite::state.imu_yaw - fluorite::state.yaw_encoder);
      if (std::abs(fluorite::state.yaw_encoder) > 1.0472) {
        lazy_state_ = LazyState::kFollow;
      }
      break;
    }
    case LazyState::kFollow: {
      fluorite::controller->base_yaw_position_pid.Update(std::copysign(0.8727, fluorite::state.yaw_encoder),
                                                         fluorite::state.yaw_encoder);
      lazily_stay_at_ = fluorite::state.imu_yaw -
                        fluorite::state.yaw_encoder;  // 小yaw轴的imu_yaw减去小yaw轴的编码器=大yaw轴的imu_yaw
      if (std::abs(fluorite::state.yaw_encoder) < 0.9599) {
        lazy_state_ = LazyState::kStay;
      }
      break;
    }
    default: {
      break;  // 不可能到这里
    }
  }
  fluorite::controller->base_yaw_speed_pid.Update(fluorite::controller->base_yaw_position_pid.value(),
                                                  fluorite::state.base_yaw_encoder_speed);
  fluorite::actuator->base_yaw.Set(fluorite::controller->base_yaw_speed_pid.value());
}
void NoForce::react(event::Control const &control) {
  fluorite::actuator->base_yaw.Enable(false);
  fluorite::controller->base_yaw_position_pid.Clear();
  fluorite::controller->base_yaw_speed_pid.Clear();
}

}  // namespace gimbal_base

}  // namespace state

}  // namespace fsm

}  // namespace fluorite

FSM_INITIAL_STATE(fluorite::fsm::Arm, fluorite::fsm::state::arm::KeyboardManual);
FSM_INITIAL_STATE(fluorite::fsm::GimbalBase, fluorite::fsm::state::gimbal_base::NoForce);