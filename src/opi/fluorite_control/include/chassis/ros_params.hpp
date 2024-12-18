
#ifndef CHASSIS_ROS_PARAMS_HPP
#define CHASSIS_ROS_PARAMS_HPP

#include <vector>
#include <array>
#include <string>

struct RosParams {
  // can net interface names
  std::string chassis_can_port{};
  std::string arm_can_port{};

  // dji motor ids
  int lf_wheel_id{};
  int rf_wheel_id{};
  int lr_wheel_id{};
  int rr_wheel_id{};
  int lf_steer_id{};
  int rf_steer_id{};
  int lr_steer_id{};
  int rr_steer_id{};

  // pid param
  std::vector<double> pid_lf_wheel_speed{};
  std::vector<double> pid_rf_wheel_speed{};
  std::vector<double> pid_lr_wheel_speed{};
  std::vector<double> pid_rr_wheel_speed{};
  std::vector<double> pid_lf_steer_speed{};
  std::vector<double> pid_rf_steer_speed{};
  std::vector<double> pid_lr_steer_speed{};
  std::vector<double> pid_rr_steer_speed{};
  std::vector<double> pid_lf_steer_position{};
  std::vector<double> pid_rf_steer_position{};
  std::vector<double> pid_lr_steer_position{};
  std::vector<double> pid_rr_steer_position{};

  // motor settings
  bool lf_wheel_reverse{};
  bool rf_wheel_reverse{};
  bool lr_wheel_reverse{};
  bool rr_wheel_reverse{};
  bool lf_steer_reverse{};
  bool rf_steer_reverse{};
  bool lr_steer_reverse{};
  bool rr_steer_reverse{};

  // configurations

  // input settings
  bool control_use_fpv_channel{};
  bool control_invert_vertical{};
  bool control_invert_horizontal{};
  double control_sensitivity_vertical{};
  double control_sensitivity_horizontal{};
};

#endif  // CHASSIS_ROS_PARAMS_HPP
