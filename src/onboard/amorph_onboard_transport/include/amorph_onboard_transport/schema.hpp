#pragma once

#include <cstdint>

struct __attribute__((packed)) Target {
  uint8_t id;
  float x;
  float y;
  float z;
  float qx;
  float qy;
  float qz;
  float qw;
};

using TargetArray = Target[10];

