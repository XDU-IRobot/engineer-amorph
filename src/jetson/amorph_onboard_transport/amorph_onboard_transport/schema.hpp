#pragma once

#include <cstdint>

struct __attribute__((packed)) TwoPointRange {
  bool range_l_valid : 1;
  bool range_r_valid : 1;
  float range_l;
  float range_r;
};