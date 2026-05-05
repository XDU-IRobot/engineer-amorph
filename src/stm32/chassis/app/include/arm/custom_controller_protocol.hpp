#pragma once

struct __attribute__((__packed__)) CustomControllerCommand {
  bool is_cartesian_command : 1;
  bool ok : 1;
  struct __attribute__((__packed__)) {
    bool a : 1;
    bool b : 1;
    bool c : 1;
    bool d : 1;
    bool reserved_1 : 1;
    bool reserved_2 : 1;
  } buttons{};
  float joint_positions[6];
};

struct __attribute__((__packed__)) CartesianCustomControllerCommand {
  bool is_cartesian_command : 1;
  bool ok : 1;
  bool btn_a : 1;
  bool btn_b : 1;
  bool reserved_1 : 1;
  bool reserved_2 : 1;
  bool reserved_3 : 1;
  bool reserved_4 : 1;
  float translation_xyz[3];
  float rotation_wxyz[4];
};
