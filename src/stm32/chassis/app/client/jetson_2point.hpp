#pragma once

#include <librm.hpp>

#include "schema.hpp"

class Jetson2Point : public rm::device::CanDevice {
 public:
  explicit Jetson2Point(rm::hal::CanInterface &can, uint16_t base_frame_id = 0x210)
      : CanDevice{can}, base_frame_id_(base_frame_id) {}

  auto data() const { return range_; }

 private:
  void RxCallback(const rm::hal::CanFrame *msg) override {
    // 只处理自己关心的帧，其余报文一律忽略
    if (msg->std_id != base_frame_id_) {
      return;
    }
    ReportStatus(kOk);
    std::memcpy(&range_, msg->data.data(), sizeof(amorph::schema::TwoPointRange));
  }

  uint16_t base_frame_id_;
  amorph::schema::TwoPointRange range_{};
};