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
  controller.chassis_solver(event.vx, event.vy, event.vw, state.lf_steer_position, state.rf_steer_position,
                            state.lr_steer_position, state.rr_steer_position);
  controller.lf_wheel_speed->Update(controller.chassis_solver.lf_wheel_speed * 1.f, state.lf_wheel_speed);
  controller.rf_wheel_speed->Update(controller.chassis_solver.rf_wheel_speed * 1.f, state.rf_wheel_speed);
  controller.lr_wheel_speed->Update(controller.chassis_solver.lr_wheel_speed * 1.f, state.lr_wheel_speed);
  controller.rr_wheel_speed->Update(controller.chassis_solver.rr_wheel_speed * 1.f, state.rr_wheel_speed);
  controller.lf_steer_position->Update(controller.chassis_solver.lf_steer_position, state.lf_steer_position);
  controller.rf_steer_position->Update(controller.chassis_solver.rf_steer_position, state.rf_steer_position);
  controller.lr_steer_position->Update(controller.chassis_solver.lr_steer_position, state.lr_steer_position);
  controller.rr_steer_position->Update(controller.chassis_solver.rr_steer_position, state.rr_steer_position);
  controller.lf_steer_speed->Update(controller.lf_steer_position->value(), state.lf_steer_speed);
  controller.rf_steer_speed->Update(controller.rf_steer_position->value(), state.rf_steer_speed);
  controller.lr_steer_speed->Update(controller.lr_steer_position->value(), state.lr_steer_speed);
  controller.rr_steer_speed->Update(controller.rr_steer_position->value(), state.rr_steer_speed);

  actuator.lf_wheel->SetCurrent(controller.lf_wheel_speed->value());
  actuator.rf_wheel->SetCurrent(controller.rf_wheel_speed->value());
  actuator.lr_wheel->SetCurrent(controller.lr_wheel_speed->value());
  actuator.rr_wheel->SetCurrent(controller.rr_wheel_speed->value());
  actuator.lf_steer->SetCurrent(controller.lf_steer_speed->value());
  actuator.rf_steer->SetCurrent(controller.rf_steer_speed->value());
  actuator.lr_steer->SetCurrent(controller.lr_steer_speed->value());
  actuator.rr_steer->SetCurrent(controller.rr_steer_speed->value());
}

}  // namespace fsm

namespace state {
namespace chassis {
void FollowGimbal::entry() {}
void NoForce::entry() {}
void NoForce::react(const event::chassis::Control &control) {
  actuator.lf_wheel->SetCurrent(0);
  actuator.rf_wheel->SetCurrent(0);
  actuator.lr_wheel->SetCurrent(0);
  actuator.rr_wheel->SetCurrent(0);
  actuator.lf_steer->SetCurrent(0);
  actuator.rf_steer->SetCurrent(0);
  actuator.lr_steer->SetCurrent(0);
  actuator.rr_steer->SetCurrent(0);
}
}  // namespace chassis
}  // namespace state

FSM_INITIAL_STATE(fsm::Chassis, state::chassis::NoForce)