
#pragma once

#include <etl/unordered_map.h>
#include <librm.hpp>

namespace rc_ch {

constexpr int LEFT_X = 3;
constexpr int LEFT_Y = 1;
constexpr int RIGHT_X = 0;
constexpr int RIGHT_Y = 2;

constexpr int LD = 4;
constexpr int RD = 5;
constexpr int LS = 6;
constexpr int RS = 7;
constexpr int SA = 8;
constexpr int SB = 9;
constexpr int SC = 10;
constexpr int SD = 11;
constexpr int SE = 12;
constexpr int SF = 13;
constexpr int SG = 14;
constexpr int SH = 15;

}  // namespace rc_ch

/**
 * @brief 基于rm::device::Sbus类对天地飞ET16s进行二次封装
 */
class WflyET16s : public rm::device::Sbus {
 public:
  WflyET16s(rm::hal::AsyncReadable &serial) : Sbus(serial) {}

  void Update() {
    for (auto &[ch, watcher] : switch_watchers_) {
      watcher.Update(channel(ch));
    }
  }

  // l-c-r 1695-1024-353, l- r+
  float left_x() const { return -(static_cast<float>(channel(rc_ch::LEFT_X)) - 1024.f) / 671.f; }
  float left_y() const { return -(static_cast<float>(channel(rc_ch::LEFT_Y)) - 1024.f) / 671.f; }
  float right_x() const { return (static_cast<float>(channel(rc_ch::RIGHT_X)) - 1024.f) / 671.f; }
  float right_y() const { return -(static_cast<float>(channel(rc_ch::RIGHT_Y)) - 1024.f) / 671.f; }

  auto &operator()(int ch) {
    // 没有错误检查，用的时候要保证ch合法
    return switch_watchers_[ch];
  }

 private:
  etl::unordered_map<int, rm::modules::SparseValueWatcher<int, true>, 8> switch_watchers_{
      {rc_ch::SA, {}},  //
      {rc_ch::SB, {}},  //
      {rc_ch::SC, {}},  //
      {rc_ch::SD, {}},  //
      {rc_ch::SE, {}},  //
      {rc_ch::SF, {}},  //
      {rc_ch::SG, {}},  //
      {rc_ch::SH, {}},  //
  };
};