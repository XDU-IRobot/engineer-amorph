#pragma once

#include <librm.hpp>

#include "schema.hpp"

class Jetson : public rm::device::CanDevice {
 public:
  explicit Jetson(rm::hal::CanInterface &can, uint16_t base_frame_id = 0x200)
      : CanDevice{can,
                  base_frame_id,
                  base_frame_id + 1,
                  base_frame_id + 2,
                  base_frame_id + 3,
                  base_frame_id + 4,
                  base_frame_id + 5,
                  base_frame_id + 6,
                  base_frame_id + 7},
        base_frame_id_(base_frame_id) {}

  const auto &data() const { return eud_; }

 private:
  void RxCallback(const rm::hal::CanFrame *msg) override {
    ReportStatus(kOk);
    const int detection_id = msg->rx_std_id - base_frame_id_;
    std::memcpy(&eud_[detection_id], msg->data.data(), sizeof(amorph::schema::EnergyUnitDetection));
  }

  uint16_t base_frame_id_;
  amorph::schema::EnergyUnitDetection eud_[8]{};
};