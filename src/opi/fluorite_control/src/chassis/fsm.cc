/**
 * @file  chassis/fsm.cc
 */

#include "chassis/fsm.hpp"

#include <iostream>
#include <unordered_map>

#include <Eigen/Dense>

#include "common/logger.hpp"

constexpr int kRcSwitchUp = 1;
constexpr int kRcSwitchMiddle = 0;
constexpr int kRcSwitchDown = -1;

fluorite_msgs::msg::ChassisState fsm::Chassis::state{};
fsm::Chassis::Actuator fsm::Chassis::actuator{};
fsm::Chassis::Controller fsm::Chassis::controller{};

namespace fsm {

// default behaviours
void Chassis::react(tinyfsm::Event const &event) {}  // 忽略不关心的event
void Chassis::react(event::chassis::Control const &control) {}

}  // namespace fsm

namespace state {
namespace chassis {

#define UpdateState()                                              \
  do {                                                             \
    switch (control.sa) {                                          \
      case kRcSwitchUp: {                                          \
        DispatchEvent(event::chassis::SwitchMode<FollowGimbal>{}); \
        break;                                                     \
      }                                                            \
      case kRcSwitchMiddle: {                                      \
        DispatchEvent(event::chassis::SwitchMode<NoForce>{});      \
        break;                                                     \
      }                                                            \
      case kRcSwitchDown: {                                        \
        DispatchEvent(event::chassis::SwitchMode<NoForce>{});      \
        break;                                                     \
      }                                                            \
    }                                                              \
  } while (0)

void FollowGimbal::entry() {}
void NoForce::entry() {}
void FollowGimbal::react(event::chassis::Control const &control) {
  const double vx = std::clamp(control.left_x + (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_D) -
                                                 control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_A)),
                               -1.0, 1.0) *
                    4000.0;
  const double vy = std::clamp(control.left_y + (control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_W) -
                                                 control.keyboard_key.at(keyboard_msgs::msg::Key::KEY_S)),
                               -1.0, 1.0) *
                    4000.0;
  const Eigen::Rotation2Dd gimbal_heading(-Chassis::state.base_yaw_encoder);
  Eigen::Vector2d control_xy(vx, vy);
  control_xy = gimbal_heading * control_xy;

  controller.chassis_orientation_position->Update(0, Chassis::state.base_yaw_encoder);

  const auto result = controller.chassis_solver.Forward(
      control_xy.y(), control_xy.x(), controller.chassis_orientation_position->value(), state.lf_steer_position,
      state.rf_steer_position, state.lr_steer_position, state.rr_steer_position);
  controller.lf_wheel_speed->Update(result.lf_wheel_speed * 1.f, state.lf_wheel_speed);
  controller.rf_wheel_speed->Update(result.rf_wheel_speed * 1.f, state.rf_wheel_speed);
  controller.lr_wheel_speed->Update(result.lr_wheel_speed * 1.f, state.lr_wheel_speed);
  controller.rr_wheel_speed->Update(result.rr_wheel_speed * 1.f, state.rr_wheel_speed);
  controller.lf_steer_position->Update(result.lf_steer_position, state.lf_steer_position);
  controller.rf_steer_position->Update(result.rf_steer_position, state.rf_steer_position);
  controller.lr_steer_position->Update(result.lr_steer_position, state.lr_steer_position);
  controller.rr_steer_position->Update(result.rr_steer_position, state.rr_steer_position);
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
  UpdateState();
}
void NoForce::react(const event::chassis::Control &control) {
  actuator.lf_wheel->SetCurrent(0);
  actuator.rf_wheel->SetCurrent(0);
  actuator.lr_wheel->SetCurrent(0);
  actuator.rr_wheel->SetCurrent(0);
  actuator.lf_steer->SetCurrent(0);
  actuator.rf_steer->SetCurrent(0);
  actuator.lr_steer->SetCurrent(0);
  actuator.rr_steer->SetCurrent(0);
  UpdateState();
}
}  // namespace chassis
}  // namespace state

FSM_INITIAL_STATE(fsm::Chassis, state::chassis::NoForce)