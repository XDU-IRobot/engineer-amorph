
#include <vector>
#include <array>
#include <string>

struct RosParams {
  // can net interface names
  std::string can_port;

  // dji motor ids
  int l_front_wheel_id;
  int l_mid_wheel_id;
  int l_rear_wheel_id;
  int l_lift_wheel_id;
  int r_front_wheel_id;
  int r_mid_wheel_id;
  int r_rear_wheel_id;
  int r_lift_wheel_id;

  // pid param
  std::vector<double> pid_l_front_speed;
  std::vector<double> pid_l_mid_speed;
  std::vector<double> pid_l_rear_speed;
  std::vector<double> pid_l_lift_speed;
  std::vector<double> pid_r_front_speed;
  std::vector<double> pid_r_mid_speed;
  std::vector<double> pid_r_rear_speed;
  std::vector<double> pid_r_lift_speed;
  std::vector<double> pid_l_lift_position;
  std::vector<double> pid_r_lift_position;

  // motor settings
  bool l_front_wheel_reverse;
  bool l_mid_wheel_reverse;
  bool l_rear_wheel_reverse;
  bool l_lift_wheel_reverse;
  bool r_front_wheel_reverse;
  bool r_mid_wheel_reverse;
  bool r_rear_wheel_reverse;
  bool r_lift_wheel_reverse;

  // input settings
  bool control_use_fpv_channel;
  bool control_invert_vertical;
  bool control_invert_horizontal;
  double control_sensitivity_vertical;
  double control_sensitivity_horizontal;
};