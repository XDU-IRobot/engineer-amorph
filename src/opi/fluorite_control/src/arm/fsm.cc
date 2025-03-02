/**
 * @file  fsm.cc
 */

#include "arm/fsm.hpp"

#include <iostream>
#include <unordered_map>

#include "common/logger.hpp"
#include "librm/device/actuator/directdrive_motor.hpp"

namespace fluorite {

fluorite_msgs::msg::ArmState state{};
std::unique_ptr<Actuator> actuator{nullptr};
std::unique_ptr<Controller> controller{nullptr};

namespace fsm {

// 忽略不关心的event
void Arm::react(tinyfsm::Event const &event) {}
void Arm::react(event::Control const &control) {}
void GimbalBase::react(tinyfsm::Event const &event) {}
void GimbalBase::react(event::Control const &control) {}

namespace state {

namespace arm {

void Debug::entry() {}
void FollowTrajectory::entry() {}
void Retracted::entry() {}
void NoForce::entry() {}

void Debug::react(event::Control const &control) {}
void FollowTrajectory::react(event::Control const &control) {}
void Retracted::react(event::Control const &control) {}
void NoForce::react(event::Control const &control) {
  fluorite::actuator->yaw.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  fluorite::actuator->pitch_1.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  fluorite::actuator->pitch_2.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  fluorite::actuator->end_yaw_1.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  fluorite::actuator->end_pitch.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  fluorite::actuator->end_yaw_2.SetCurrent(0);
  fluorite::controller->trajectory_tracker.ClearPid();
}

}  // namespace arm

namespace gimbal_base {

void FollowChassis::entry() { fluorite::actuator->base_yaw.Enable(true); }
void FollowArm::entry() { fluorite::actuator->base_yaw.Enable(true); }
void FollowArmLazy::entry() {
  lazy_state_ = LazyState::kStay;
  fluorite::actuator->base_yaw.Enable(true);
}
void NoForce::entry() {}

void FollowChassis::react(event::Control const &control) {
  fluorite::controller->base_yaw_position_pid.Update(0, fluorite::state.base_yaw_encoder);
  fluorite::controller->base_yaw_speed_pid.Update(0, fluorite::state.base_yaw_encoder_speed);
  fluorite::actuator->base_yaw.Set(fluorite::controller->base_yaw_position_pid.value());
}
void FollowArm::react(event::Control const &control) {
  fluorite::controller->base_yaw_position_pid.Update(0, fluorite::state.yaw_encoder);
  fluorite::controller->base_yaw_speed_pid.Update(0, fluorite::state.base_yaw_encoder_speed);
  fluorite::actuator->base_yaw.Set(fluorite::controller->base_yaw_position_pid.value());
}
double FollowArmLazy::lazily_stay_at_ = 0;
FollowArmLazy::LazyState FollowArmLazy::lazy_state_ = LazyState::kStay;
void FollowArmLazy::react(event::Control const &control) {
  // 如果yaw轴编码器超过60度(1.0472rad)，就把error控制到50度(0.8727rad)，误差小于50度之后大yaw轴进入lazy状态，大yaw轴停留在当前位置，直到小yaw轴编码器再次超过60度，lazy状态解除，再次把error控制到50度
  switch (lazy_state_) {
    case LazyState::kStay: {
      if (std::abs(fluorite::state.yaw_encoder) > 1.0472) {
        fluorite::controller->base_yaw_position_pid.Update(lazily_stay_at_, fluorite::state.base_yaw_encoder);
        lazy_state_ = LazyState::kFollow;
      }
      break;
    }
    case LazyState::kFollow: {
      fluorite::controller->base_yaw_position_pid.Update(std::copysign(0.8727, fluorite::state.yaw_encoder),
                                                         fluorite::state.yaw_encoder);
      lazily_stay_at_ = fluorite::state.base_yaw_encoder;
      if (std::abs(fluorite::state.yaw_encoder) < 0.8727) {
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

FSM_INITIAL_STATE(fluorite::fsm::Arm, fluorite::fsm::state::arm::NoForce)
FSM_INITIAL_STATE(fluorite::fsm::GimbalBase, fluorite::fsm::state::gimbal_base::NoForce)