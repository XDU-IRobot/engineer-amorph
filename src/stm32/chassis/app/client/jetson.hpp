#pragma once

#include <librm.hpp>

#include "schema.hpp"

class Jetson : public rm::device::CanDevice {
 public:
  explicit Jetson(rm::hal::CanInterface &can, uint16_t base_frame_id = 0x200)
      : CanDevice{can}, base_frame_id_(base_frame_id) {}

  void FeedbackJs(const amorph::schema::JointState &js) const {
    rm::hal::CanFrame tx{.std_id = 0x100, .dlc = rm::hal::CanFrame::kDlc64, .is_fd_frame = true};
    std::memcpy(tx.data.data(), &js, sizeof(js));
    can_->Write(tx);
  }

 private:
  void RxCallback(const rm::hal::CanFrame *msg) override {
    // 只处理本设备负责的 8 个连续 ID，其余报文一律忽略
    if ((msg->std_id < base_frame_id_) || (msg->std_id >= static_cast<uint16_t>(base_frame_id_ + 8))) {
      return;
    }
    ReportStatus(kOk);
  }

  uint16_t base_frame_id_;
};