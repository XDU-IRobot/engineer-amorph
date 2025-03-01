/**
 * @file  fsm.cc
 */

#include "arm/fsm.hpp"

#include <iostream>
#include <unordered_map>

#include "common/logger.hpp"

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
void NoForce::react(event::Control const &control) {}

}  // namespace arm

namespace gimbal_base {

void FollowChassis::entry() {}
void FollowArm::entry() {}
void FollowArmLazy::entry() {}
void NoForce::entry() {}

void FollowChassis::react(event::Control const &control) {}
void FollowArm::react(event::Control const &control) {}
void FollowArmLazy::react(event::Control const &control) {}
void NoForce::react(event::Control const &control) {}

}  // namespace gimbal_base

}  // namespace state

}  // namespace fsm

}  // namespace fluorite

FSM_INITIAL_STATE(fluorite::fsm::Arm, fluorite::fsm::state::arm::NoForce)
FSM_INITIAL_STATE(fluorite::fsm::GimbalBase, fluorite::fsm::state::gimbal_base::NoForce)