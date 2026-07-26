#pragma once

#include "action_sequencer.hpp"

namespace arm_sequence {

inline ActionSequence LowReset(const std::array<float, 6> &current_pos) {
  using namespace rm::modules::angle_literals;
  const auto rise_j2 = [&] {
    std::array<float, 6> p{current_pos};
    p[1] = (180_deg).rad() - std::atan2f(84.5f, 415.f) - (42_deg).rad();
    p[2] = -(180_deg).rad() + std::atan2f(84.5f, 415.f) + (30_deg).rad();
    return p;
  }();
  const auto center_remaining = [&] {
    std::array<float, 6> p;
    p.fill(0.f);
    p[1] = rise_j2[1];
    p[2] = rise_j2[2];
    return p;
  }();
  ActionSequence seq;
  seq.GoTo(rise_j2)  //
      .GoTo(center_remaining);
  return seq;
}

inline ActionSequence HiReset(const std::array<float, 6> &current_pos) {
  using namespace rm::modules::angle_literals;
  const auto center_j1j2j3 = [&] {
    auto p{current_pos};
    p[0] = 0.f;
    p[1] = (180_deg).rad() - std::atan2f(84.5f, 415.f) - (42_deg).rad();
    p[2] = -(180_deg).rad() + std::atan2f(84.5f, 415.f) + (30_deg).rad();
    return p;
  }();
  const auto center_remaining = [&] {
    std::array<float, 6> p;
    p.fill(0.f);
    p[1] = center_j1j2j3[1];
    p[2] = center_j1j2j3[2];
    return p;
  }();
  ActionSequence seq;
  seq.GoTo(center_j1j2j3)  //
      .GoTo(center_remaining);
  return seq;
}

inline ActionSequence Reset(const std::array<float, 6> &current_pos) {
  if (current_pos[1] > (130_deg).rad()) {
    return LowReset(current_pos);
  } else {
    return HiReset(current_pos);
  }
}

inline ActionSequence HalfFuckOff(const std::array<float, 6> &current_pos) {
  ActionSequence seq;
  seq.GoTo(current_pos);
  auto reset = Reset(current_pos);
  auto next = reset.steps().back().positions;
  next[2] += (55_deg).rad();
  next[4] = (45_deg).rad();
  seq.GoTo(next);
  return seq;
}

inline ActionSequence FuckOff(const std::array<float, 6> &current_pos) {
  ActionSequence seq;
  seq.GoTo(current_pos);
  auto reset = Reset(current_pos);
  auto next = reset.steps().back().positions;
  next[2] += (70_deg).rad();
  seq.GoTo(next);
  return seq;
}

inline ActionSequence Retract(const std::array<float, 6> &current_pos, bool left_arm) {
  auto reset = Reset(current_pos);
  reset.GoTo({0.02f,                                                            //
              (180_deg).rad() - std::atan2f(84.5f, 472.254f) - (15_deg).rad(),  //
              -2.85f, 0.f, 0.f, 0.f});
  if (left_arm) {
    reset.GoTo({0.0f, (180_deg).rad() - std::atan2f(84.5f, 472.254f), -2.85f, 0.f, -(7_deg).rad(), 0.f});
    // reset.GoTo({-0.14f, 2.935f, -2.85f, 0.f, -(60_deg).rad(), 0.f});
  } else {
    reset.GoTo({-0.03f, (180_deg).rad() - std::atan2f(84.5f, 472.254f), -2.85f, 0.f, -(7_deg).rad(), 0.f});
    // reset.GoTo({0.07f, 2.935f, -2.85f, 0.f, -(60_deg).rad(), 0.f});
  }
  return reset;
}

inline ActionSequence LeftTake2(const std::array<float, 6> &current_pos) {
  auto reset = Reset(current_pos);
  reset.GripperOpen(true);
  reset.GoTo({-0.647877216f, 2.47491407f, -1.93327177f, 1.25562668f, -0.703772306f, 0.0444908142f});
  reset.GoTo({-0.536182404f, 2.26504326f, -1.84858406f, 1.67372417f, -0.631962061f, -0.457949162f});
  reset.GripperOpen(false);
  reset.GoTo({0.000719070435f, 2.33934665f, -1.97637832f, 1.04390812f, -0.0873904228f, 0.0391212702f});
  return reset;
}

inline ActionSequence RightTake3(const std::array<float, 6> &current_pos) {
  auto reset = Reset(current_pos);
  reset.GripperOpen(true);
  reset.GoTo({0.721701145f, 2.64164138f, -2.12477779f, -0.813453674f, -0.548646212f, 0.758646011f});
  reset.GoTo({0.60454154f, 2.32314348f, -2.01241207f, -1.09254646f, -0.448360682f, 1.10229969f});
  reset.GripperOpen(false);
  reset.GoTo({0.649027586f, 2.34212685f, -2.28316355f, -1.76185179f, -0.602624178f, 1.90236843f});
  return reset;
}

inline ActionSequence RightTake4(const std::array<float, 6> &current_pos) {
  auto reset = Reset(current_pos);
  reset.GripperOpen(true);
  reset.GoTo({0.530525923f, 2.5837326f, -2.21461296f, -2.19712591f, -0.960526228f, 0.82077992f});
  reset.GoTo({0.259965897f, 2.33138895f, -1.91644084f, -1.64776039f, -0.586325169f, 0.489399582f});
  // reset.GripperOpen(false);
  // reset.GoTo({0.0784738064f, 2.51709938f, -2.02674532f, 2.2558012f, 0.217972517f, -0.263876915f});
  return reset;
}

inline ActionSequence LeftStore(const std::array<float, 6> &current_pos) {
  ActionSequence seq;
  seq.GoTo(current_pos);
  seq.GripperOpen(true);
  seq.GoTo({0.530525923f, 2.5837326f, -2.21461296f, -2.19712591f, -0.960526228f, 0.82077992f});
  seq.GoTo({0.259965897f, 2.33138895f, -1.91644084f, -1.64776039f, -0.586325169f, 0.489399582f});
  // reset.GripperOpen(false);
  // reset.GoTo({0.0784738064f, 2.51709938f, -2.02674532f, 2.2558012f, 0.217972517f, -0.263876915f});
  return seq;
}

}  // namespace arm_sequence