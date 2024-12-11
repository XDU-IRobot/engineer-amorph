/**
 * @file  fsm.cc
 */

#include "chassis/fsm.hpp"

#include <iostream>
#include <unordered_map>

#include "common/logger.hpp"

using namespace rm::modules::algorithm::utils;

fluorite_msgs::msg::ChassisState fsm::Chassis::state{};
fsm::Chassis::Actuator fsm::Chassis::actuator{};
fsm::Chassis::Controller fsm::Chassis::controller{};

namespace fsm {

// default behaviours
void Chassis::react(tinyfsm::Event const &event) {}  // 忽略不关心的event
void Chassis::react(event::chassis::Control const &event) {
  controller.chassis_solver.Forward(event.vx, event.vy, event.vw);
  controller.l_front_speed->Update(controller.chassis_solver.wheel_speeds_.l_front, state.l_front_speed);
  controller.r_front_speed->Update(controller.chassis_solver.wheel_speeds_.r_front, state.r_front_speed);
  controller.l_mid_speed->Update(controller.chassis_solver.wheel_speeds_.l_mid, state.l_mid_speed);
  controller.r_mid_speed->Update(controller.chassis_solver.wheel_speeds_.r_mid, state.r_mid_speed);
  controller.l_rear_speed->Update(controller.chassis_solver.wheel_speeds_.l_rear, state.l_rear_speed);
  controller.r_rear_speed->Update(controller.chassis_solver.wheel_speeds_.r_rear, state.r_rear_speed);
  actuator.l_front_wheel->SetCurrent(controller.l_front_speed->value());
  actuator.r_front_wheel->SetCurrent(controller.r_front_speed->value());
  actuator.l_mid_wheel->SetCurrent(controller.l_mid_speed->value());
  actuator.r_mid_wheel->SetCurrent(controller.r_mid_speed->value());
  actuator.l_rear_wheel->SetCurrent(controller.l_rear_speed->value());
  actuator.r_rear_wheel->SetCurrent(controller.r_rear_speed->value());

  if (!this->is_in_state<state::chassis::Adaptive>()) {
    controller.l_lift_position->Update(state.target_lift_encoder, state.l_lift_position);
    controller.l_lift_speed->Update(controller.l_lift_position->value(), state.l_lift_speed);
    controller.r_lift_position->Update(state.target_lift_encoder, state.r_lift_position);
    controller.r_lift_speed->Update(controller.r_lift_position->value(), state.r_lift_speed);
    actuator.l_lift->SetCurrent(controller.l_lift_speed->value());
    actuator.r_lift->SetCurrent(controller.r_lift_speed->value());
  }
}

}  // namespace fsm

namespace state {
namespace chassis {
void Lift::entry() { state.target_lift_encoder = -105000; }
void Lower::entry() { state.target_lift_encoder = 90000; }
void Neutral::entry() { state.target_lift_encoder = 0; }
void Adaptive::entry() { state.target_lift_encoder = 0; }
void NoForce::entry() { state.target_lift_encoder = 0; }
void NoForce::react(const event::chassis::Control &control) {
  actuator.l_front_wheel->SetCurrent(0);
  actuator.r_front_wheel->SetCurrent(0);
  actuator.l_mid_wheel->SetCurrent(0);
  actuator.r_mid_wheel->SetCurrent(0);
  actuator.l_rear_wheel->SetCurrent(0);
  actuator.r_rear_wheel->SetCurrent(0);
  actuator.l_lift->SetCurrent(0);
  actuator.r_lift->SetCurrent(0);
}
}  // namespace chassis
}  // namespace state

FSM_INITIAL_STATE(fsm::Chassis, state::chassis::NoForce)