#pragma once

#include <atomic>
#include <cstring>
#include <mutex>
#include <optional>

#include <librm.hpp>

#include "amorph_transport/schema.hpp"

namespace amorph {

/**
 * @brief STM32 CAN 设备，接收下位机 JointState 反馈 (stdid=0x100, FD 帧)
 */
class STM32 : public rm::device::CanDevice {
 public:
  explicit STM32(rm::hal::CanInterface &can)
      : CanDevice(can, 0x100) {}

  /// 获取最新一帧 JointState（线程安全），无新数据时返回 nullopt
  std::optional<schema::JointState> GetLatest() {
    std::lock_guard lock(mutex_);
    if (has_new_) {
      has_new_ = false;
      return latest_;
    }
    return std::nullopt;
  }

 private:
  void RxCallback(const rm::hal::CanFrame *msg) override {
    // 校验 DLC 不小于 JointState 大小
    constexpr auto kExpectedSize = sizeof(schema::JointState);
    if (msg->dlc < kExpectedSize) {
      return;
    }

    schema::JointState state;
    std::memcpy(&state, msg->data.data(), kExpectedSize);

    {
      std::lock_guard lock(mutex_);
      latest_ = state;
      has_new_ = true;
    }
  }

  std::mutex mutex_;
  schema::JointState latest_{};
  bool has_new_{false};
};

}  // namespace amorph
