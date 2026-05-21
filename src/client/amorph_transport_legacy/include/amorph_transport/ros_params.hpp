#pragma once

#include <string>

struct RosParams {
  struct {
    std::string can_port;
    int mosi_frame_id;
    int miso_frame_id;
  } rpc;
  std::string arm_action_server_name;
  std::string gripper_action_server_name;
  std::string joint_state_topic;
  std::vector<std::string> joint_names;
};