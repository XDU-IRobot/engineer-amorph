
#pragma once

#include <string>
#include <vector>

namespace pick_place_task {

struct Params {
  int max_solutions;
  std::string arm_group_name;
  std::string eef_name;
  std::string hand_group_name;
  std::string hand_frame;
  std::string hand_open_pose;
  std::string hand_close_pose;
  std::string arm_home_pose;
  std::string world_frame;
  std::string table_reference_frame;
  std::string object_reference_frame;
  std::string surface_link;
  std::string object_name;
  std::vector<double> object_dimensions;
  std::vector<double> object_pose;
  bool spawn_table;
  std::string table_name;
  std::vector<double> table_dimensions;
  std::vector<double> table_pose;
  std::vector<double> grasp_frame_transform;
  std::vector<double> place_pose;
  double place_surface_offset;
  double approach_object_min_dist;
  double approach_object_max_dist;
  double lift_object_min_dist;
  double lift_object_max_dist;
};

}  // namespace pick_place_task