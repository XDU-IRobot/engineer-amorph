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

  void FeedbackJs(const amorph::schema::JointState &js) const {
    can_->Write(0x100, reinterpret_cast<const uint8_t *>(&js), sizeof(js));
  }

 private:
  void RxCallback(const rm::hal::CanFrame *msg) override { ReportStatus(kOk); }

  uint16_t base_frame_id_;
};