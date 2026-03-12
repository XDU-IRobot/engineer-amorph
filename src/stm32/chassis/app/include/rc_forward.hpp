/**
 * @file  rc_forward.hpp
 * @brief 处理遥控器数据在CANFD总线上转发的发送端和接收端
 */

#pragma once

#include <librm.hpp>
#include <etl/unordered_map.h>

/**
 * @brief 遥控器数据转发器的发送端，负责把遥控器数据打包成CAN帧发送到总线上
 */
class RcForwarder {
 public:
  constexpr static int16_t kFrameId = 0x5e;

  explicit RcForwarder(rm::hal::CanInterface &can) : can_{&can} {}

  void Write(const int16_t channels[16]) const {
    can_->Write(kFrameId, reinterpret_cast<const uint8_t *>(channels), 16 * sizeof(int16_t));
  }

 private:
  rm::hal::CanInterface *can_;
};

/**
 * @brief 遥控器数据转发器的接收端，负责接收CAN总线上的遥控器数据，并提供一些便利的接口
 */
class RcReceiver : public rm::device::CanDevice {
 public:
  /**
   * @brief 遥控器通道枚举
   */
  enum ch {
    kLeftX = 3,
    kLeftY = 1,
    kRightX = 0,
    kRightY = 2,
    kLd = 4,
    kRd = 5,
    kLs = 6,
    kRs = 7,
    kSa = 8,
    kSb = 9,
    kSc = 10,
    kSd = 11,
    kSe = 12,
    kSf = 13,
    kSg = 14,
    kSh = 15,
  };

  explicit RcReceiver(rm::hal::CanInterface &can) : CanDevice{can, RcForwarder::kFrameId} {}

  void UpdateSwitchWatchers() {
    for (auto &[ch, watcher] : switch_watchers_) {
      watcher.Update(channels_[ch]);
    }
  }

  // l-c-r 1695-1024-353, l- r+
  float left_x() const { return -(static_cast<float>(channels_[ch::kLeftX]) - 1024.f) / 671.f; }
  float left_y() const { return -(static_cast<float>(channels_[ch::kLeftY]) - 1024.f) / 671.f; }
  float right_x() const { return (static_cast<float>(channels_[ch::kRightX]) - 1024.f) / 671.f; }
  float right_y() const { return -(static_cast<float>(channels_[ch::kRightY]) - 1024.f) / 671.f; }

  auto &operator()(int ch) {
    // 没有错误检查，用的时候要保证ch合法
    return switch_watchers_[ch];
  }

 private:
  void RxCallback(const rm::hal::CanFrame *msg) override {
    ReportStatus(kOk);
    std::memcpy(channels_, msg->data.data(), sizeof(channels_));
  }

  int16_t channels_[16]{};
  etl::unordered_map<int, rm::modules::SparseValueWatcher<int, true>, 8> switch_watchers_{
      {ch::kSa, {}},  //
      {ch::kSb, {}},  //
      {ch::kSc, {}},  //
      {ch::kSd, {}},  //
      {ch::kSe, {}},  //
      {ch::kSf, {}},  //
      {ch::kSg, {}},  //
      {ch::kSh, {}},  //
  };  ///< 拨杆通道的值监视器，拨杆被拨动时触发用户注册的回调函数
};