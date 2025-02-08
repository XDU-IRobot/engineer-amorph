
#ifndef CHASSIS_ROS_PARAMS_HPP
#define CHASSIS_ROS_PARAMS_HPP

#include <string>
#include <vector>

struct RosParams {
  // can net interface names
  std::string arm_can_port{};

  // motor id
  int base_yaw_id{};
  short yaw_master_id{};
  short yaw_slave_id{};
  short pitch_1_master_id{};
  short pitch_1_slave_id{};
  short pitch_2_master_id{};
  short pitch_2_slave_id{};
  short end_yaw_1_master_id{};
  short end_yaw_1_slave_id{};
  short end_pitch_master_id{};
  short end_pitch_slave_id{};
  int end_yaw_2_id{};

  // pid param
  std::vector<double> pid_base_yaw_speed{};
  std::vector<double> pid_yaw_speed{};
  std::vector<double> pid_pitch_1_speed{};
  std::vector<double> pid_pitch_2_speed{};
  std::vector<double> pid_end_yaw_1_speed{};
  std::vector<double> pid_end_pitch_speed{};
  std::vector<double> pid_end_yaw_2_speed{};
  std::vector<double> pid_base_yaw_position{};
  std::vector<double> pid_yaw_position{};
  std::vector<double> pid_pitch_1_position{};
  std::vector<double> pid_pitch_2_position{};
  std::vector<double> pid_end_yaw_1_position{};
  std::vector<double> pid_end_pitch_position{};
  std::vector<double> pid_end_yaw_2_position{};

  // encoder offsets
  double base_yaw_encoder_zero_offset{};
  double yaw_encoder_zero_offset{};
  double pitch_1_encoder_zero_offset{};
  double pitch_2_encoder_zero_offset{};
  double end_yaw_1_encoder_zero_offset{};
  double end_pitch_encoder_zero_offset{};
  double end_yaw_2_encoder_zero_offset{};

  // motor settings
  bool base_yaw_reverse{};
  bool yaw_reverse{};
  bool pitch_1_reverse{};
  bool pitch_2_reverse{};
  bool end_yaw_1_reverse{};
  bool end_pitch_reverse{};
  bool end_yaw_2_reverse{};

  // gpio settings
  int sucker_out_valve_pin{};
  int sucker_in_valve_pin{};
  int sucker_vaccum_pump_pin{};
  int storage_left_valve_pin{};
  int storage_right_valve_pin{};
  int storage_vaccum_pump_pin{};

  // input settings
  bool control_use_fpv_channel{};  // 使用图传链路控制
  bool control_invert_vertical{};
  bool control_invert_horizontal{};
  double control_sensitivity_vertical{};
  double control_sensitivity_horizontal{};
};

#endif  // CHASSIS_ROS_PARAMS_HPP
