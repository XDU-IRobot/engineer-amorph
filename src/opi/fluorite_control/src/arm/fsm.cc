/**
 * @file  fsm.cc
 */

#include "arm/fsm.hpp"

#include <iostream>
#include <unordered_map>

#include "common/logger.hpp"

using namespace rm::modules::algorithm::utils;

fluorite_msgs::msg::ArmState fsm::Arm::state{};
fsm::Arm::Actuator fsm::Arm::actuator{};
fsm::Arm::Controller fsm::Arm::controller{};

namespace fsm {

// default behaviours
void Arm::react(tinyfsm::Event const &event) {}  // 忽略不关心的event
void Arm::react(event::arm::Control const &event) {}

}  // namespace fsm

namespace state {
namespace arm {
void Debug::entry() {}
void NoForce::entry() {}
void NoForce::react(const event::arm::Control &control) {}
}  // namespace arm
}  // namespace state

FSM_INITIAL_STATE(fsm::Arm, state::arm::NoForce)