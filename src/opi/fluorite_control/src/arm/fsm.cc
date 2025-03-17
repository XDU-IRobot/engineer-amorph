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
  fluorite::actuator->base_yaw.Enable(enable);
  const auto dm_enable = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
  fluorite::actuator->yaw.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  fluorite::actuator->pitch_1.SendInstruction(dm_enable);
  fluorite::actuator->pitch_2.SendInstruction(dm_enable);
  fluorite::actuator->end_yaw_1.SendInstruction(dm_enable);
  fluorite::actuator->end_pitch.SendInstruction(dm_enable);
}

void Arm::CalcPidAndExecute() {
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

namespace state {

namespace arm {

void Debug::entry() { Arm::EnableActuators(true); }
void KeyboardManual::entry() {
  Arm::EnableActuators(true);
  fluorite::controller->move_group_request_mutex.try_lock();
  fluorite::controller->move_group_request_thread_pool_.enqueue([&] {
    // 获取当前机械臂末端的位姿，设置为目标位姿
    fluorite::state.target_end_effector_pose = fluorite::controller->move_group.getCurrentPose();
    fluorite::controller->move_group_request_mutex.unlock();
  });
}
void CustomCtrlManual::entry() {
  Arm::EnableActuators(true);
  fluorite::controller->move_group_request_thread_pool_.enqueue([&] {
    // 获取当前机械臂末端的位姿，设置为目标位姿
    fluorite::state.target_end_effector_pose = fluorite::controller->move_group.getCurrentPose();
    fluorite::controller->move_group_request_mutex.unlock();
  });
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
    joint_positions.positions[0] += control.left_y * 0.001;
  } else if (control.se == kRcSwitchMiddle) {
    joint_positions.positions[1] += control.left_y * 0.001;
  } else {
    if (control.sg == kRcSwitchUp) {
      joint_positions.positions[2] += control.left_y * 0.001;
    } else if (control.sg == kRcSwitchMiddle) {
      joint_positions.positions[3] += control.left_y * 0.001;
    } else {
      if (control.sh == kRcSwitchUp) {
        joint_positions.positions[4] += control.left_y * 0.001;
      } else if (control.sh == kRcSwitchDown) {
        joint_positions.positions[5] += control.left_y * 0.001;
      }
    }
  }
  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(joint_positions);
  Arm::CalcPidAndExecute();
  std::printf("output: ");
  const auto tau = fluorite::controller->trajectory_tracker.output_torque();
  for (const auto val : tau) {
    std::printf("%f ", val);
  }
  std::printf("target position: ");
  for (const auto jp : joint_positions.positions) {
    std::printf("%f ", jp);
  }
  std::printf(" | ");
  std::printf("\n");
  UpdateState();
}
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
    // 并且上一轮计算结束（锁被释放），才计算一次新的目标关节位置，否则还是按TrajectoryTracker中的参考轨迹继续运动
    if (fluorite::controller->move_group_request_mutex.try_lock()) {
      fluorite::controller->move_group_request_thread_pool_.enqueue([&] {  // 异步计算ik，防止阻塞控制循环
        try {
          const auto target_pose = [&] {                                   // 根据键盘输入设置目标位置
            auto ret = fluorite::controller->move_group.getCurrentPose();  // 获取当前机械臂末端的位姿

            const double delta_x = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_D) -
                                   control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_A) * 0.02;
            const double delta_y = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_S) -
                                   control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_W) * 0.02;
            const double delta_z = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_R) -
                                   control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_F) * 0.02;
            const double delta_yaw = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_U) -
                                     control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_O) * 0.02;
            double delta_pitch{0}, delta_roll{0};
            if (control.keyboard_modifier & keyboard_msgs::msg::Key::MODIFIER_LSHIFT) {
              delta_pitch = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_I) -
                            control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_K) * 0.02;
              delta_roll = control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_J) -
                           control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_L) * 0.02;
            }

            const Eigen::Quaterniond q(fluorite::state.target_end_effector_pose.pose.orientation.w,
                                       fluorite::state.target_end_effector_pose.pose.orientation.x,
                                       fluorite::state.target_end_effector_pose.pose.orientation.y,
                                       fluorite::state.target_end_effector_pose.pose.orientation.z);
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

  Arm::CalcPidAndExecute();
  UpdateState();
}
void CustomCtrlManual::react(event::Control const &control) {
  // TODO
}
void FollowExternalTrajectory::react(event::Control const &control) {
  Arm::CalcPidAndExecute();

  UpdateState();
}
void Retracted::react(event::Control const &control) {
  fluorite::state.imu_yaw_target += control.left_x;
  const auto kReferenceTraj = [&] {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {fluorite::state.imu_yaw_target, 0, 0, 0, 0, 0};
    p.velocities = {0, 0, 0, 0, 0, 0};
    return p;
  }();

  fluorite::controller->trajectory_tracker.SwitchReferenceTrajectory(kReferenceTraj);
  Arm::CalcPidAndExecute();

  // 状态机模式更新
  if (control.sd != kRcSwitchUp) {
    DispatchEvent(event::SwitchMode<NoForce>{});
    return;
  }

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

FSM_INITIAL_STATE(fluorite::fsm::Arm, fluorite::fsm::state::arm::NoForce);
FSM_INITIAL_STATE(fluorite::fsm::GimbalBase, fluorite::fsm::state::gimbal_base::NoForce);