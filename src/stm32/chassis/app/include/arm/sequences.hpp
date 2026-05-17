#pragma once

#include "action_sequencer.hpp"

namespace arm_sequence {

inline ActionSequence LowReset(const std::array<float, 6> &current_pos) {
  using namespace rm::modules::angle_literals;
  const auto rise_j2 = [&] {
    std::array<float, 6> p{current_pos};
    p[1] = (180_deg).rad() - std::atan2f(84.5f, 415.f) - (15_deg).rad();
    return p;
  }();
  const auto rise_j3 = [&] {
    auto p{rise_j2};
    p[2] = -(180_deg).rad() + std::atan2f(84.5f, 415.f) + (20_deg).rad();
    return p;
  }();
  const auto center_remaining = [&] {
    std::array<float, 6> p;
    p.fill(0.f);
    p[1] = rise_j2[1];
    p[2] = rise_j3[2];
    return p;
  }();
  ActionSequence seq;
  seq.GoTo(rise_j2)   //
      .GoTo(rise_j3)  //
      .GoTo(center_remaining);
  return seq;
}

inline ActionSequence HiReset(const std::array<float, 6> &current_pos) {
  using namespace rm::modules::angle_literals;
  const auto rise_j3_center_j1 = [&] {
    auto p{current_pos};
    p[0] = 0.f;
    p[2] += (35_deg).rad();
    return p;
  }();
  const auto retract_j2j3_center_remaining = [&] {
    std::array<float, 6> p;
    p.fill(0.f);
    p[1] = (180_deg).rad() - std::atan2f(84.5f, 415.f) - (15_deg).rad();
    p[2] = -(180_deg).rad() + std::atan2f(84.5f, 415.f) + (20_deg).rad();
    return p;
  }();
  ActionSequence seq;
  seq.GoTo(rise_j3_center_j1)  //
      .GoTo(retract_j2j3_center_remaining);
  return seq;
}

inline ActionSequence Reset(const std::array<float, 6> &current_pos) {
  if (current_pos[1] > (130_deg).rad()) {
    return LowReset(current_pos);
  } else {
    return HiReset(current_pos);
  }
}

inline ActionSequence Retract(const std::array<float, 6> &current_pos) {
  auto reset = Reset(current_pos);
  reset.GoTo({0.02f,                                                         //
              (180_deg).rad() - std::atan2f(84.5f, 415.f) - (15_deg).rad(),  //
              -2.85f, 0.f, -1.564f, 0.f});
  reset.GoTo({0.02f, 2.935f, -2.85f, 0.f, -1.564f, 0.f});
  reset.GoTo({0.f, 2.935f, -2.85f, 0.f, -1.564f, 0.f});
  return reset;
}
}  // namespace arm_sequence