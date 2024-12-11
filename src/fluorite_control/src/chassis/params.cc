
#include "chassis/node.hpp"

void FluoriteChassis::InitParameters() {
  this->get_parameter("can_port", params_.can_port);

  // dji motor ids
  this->get_parameter("l_front_wheel_id", params_.l_front_wheel_id);
  this->get_parameter("l_mid_wheel_id", params_.l_mid_wheel_id);
  this->get_parameter("l_rear_wheel_id", params_.l_rear_wheel_id);
  this->get_parameter("l_lift_wheel_id", params_.l_lift_wheel_id);
  this->get_parameter("r_front_wheel_id", params_.r_front_wheel_id);
  this->get_parameter("r_mid_wheel_id", params_.r_mid_wheel_id);
  this->get_parameter("r_rear_wheel_id", params_.r_rear_wheel_id);
  this->get_parameter("r_lift_wheel_id", params_.r_lift_wheel_id);

  // pid param
  this->get_parameter("pid_l_front_speed", params_.pid_l_front_speed);
  this->get_parameter("pid_l_mid_speed", params_.pid_l_mid_speed);
  this->get_parameter("pid_l_rear_speed", params_.pid_l_rear_speed);
  this->get_parameter("pid_l_lift_speed", params_.pid_l_lift_speed);
  this->get_parameter("pid_r_front_speed", params_.pid_r_front_speed);
  this->get_parameter("pid_r_mid_speed", params_.pid_r_mid_speed);
  this->get_parameter("pid_r_rear_speed", params_.pid_r_rear_speed);
  this->get_parameter("pid_r_lift_speed", params_.pid_r_lift_speed);
  this->get_parameter("pid_l_lift_position", params_.pid_l_lift_speed);
  this->get_parameter("pid_r_lift_position", params_.pid_r_lift_speed);

  // motor settings
  this->get_parameter("l_front_wheel_reverse", params_.l_front_wheel_reverse);
  this->get_parameter("l_mid_wheel_reverse", params_.l_mid_wheel_reverse);
  this->get_parameter("l_rear_wheel_reverse", params_.l_rear_wheel_reverse);
  this->get_parameter("l_lift_wheel_reverse", params_.l_lift_wheel_reverse);
  this->get_parameter("r_front_wheel_reverse", params_.r_front_wheel_reverse);
  this->get_parameter("r_mid_wheel_reverse", params_.r_mid_wheel_reverse);
  this->get_parameter("r_rear_wheel_reverse", params_.r_rear_wheel_reverse);
  this->get_parameter("r_lift_wheel_reverse", params_.r_lift_wheel_reverse);

  // configurations
  this->get_parameter("lift_encoder_delta", params_.lift_encoder_delta);
  this->get_parameter("lower_encoder_delta", params_.lower_encoder_delta);

  // input settings
  this->get_parameter("control_use_fpv_channel", params_.control_use_fpv_channel);
  this->get_parameter("control_invert_vertical", params_.control_invert_vertical);
  this->get_parameter("control_invert_horizontal", params_.control_invert_horizontal);
  this->get_parameter("control_sensitivity_vertical", params_.control_sensitivity_vertical);
  this->get_parameter("control_sensitivity_horizontal", params_.control_sensitivity_horizontal);
}