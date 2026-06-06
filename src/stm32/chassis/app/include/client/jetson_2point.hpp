#pragma once

#include <librm.hpp>

#include "schema.hpp"

class Jetson2Point : public rm::device::CanDevice {
 public:
  explicit Jetson2Point(rm::hal::CanInterface &can, uint16_t base_frame_id = 0x210)
      : CanDevice{
            can,
            base_frame_id,
        },
        base_frame_id_(base_frame_id) {}

  auto data() const { return range_; }

 private:
  void RxCallback(const rm::hal::CanFrame *msg) override {
    ReportStatus(kOk);
    const int detection_id = msg->rx_std_id - base_frame_id_;
    std::memcpy(&range_, msg->data.data(), sizeof(amorph::schema::TwoPointRange));
  }

  uint16_t base_frame_id_;
  amorph::schema::TwoPointRange range_{};
};