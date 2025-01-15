
#ifndef CHASSIS_ROS_PARAMS_HPP
#define CHASSIS_ROS_PARAMS_HPP

#include <string>

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
};

#endif  // CHASSIS_ROS_PARAMS_HPP
