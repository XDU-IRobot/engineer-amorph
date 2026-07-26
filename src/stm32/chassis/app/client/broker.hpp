#pragma once

#include <librm.hpp>

#include "schema.hpp"

class CustomClientBroker : public rm::device::Device {
 public:
  template <rm::device::RefereeRevision RefereeRev>
  explicit CustomClientBroker(rm::device::Referee<RefereeRev> &ref) {
    ref.AttachCallback([&](uint16_t cmd_id, uint8_t seq) {
      UNUSED(seq);
      if (cmd_id == 0x311) {
        const auto &data = ref.data().robot_custom_data_3.data;
        switch (data[0]) {
          case amorph::schema::JoyCompressed{}.cmd_id: {
            ReportStatus(kOk);
            amorph::schema::JoyCompressed compressed;
            std::memcpy(&compressed, &data, sizeof(amorph::schema::JoyCompressed));
            joy_ = amorph::schema::Joy(compressed);
            if (on_joy_recv_) {
              on_joy_recv_(joy_);
            }
            break;
          }
          case amorph::schema::ArmCommandCompressed{}.cmd_id: {
            ReportStatus(kOk);
            if (HAL_GetTick() - refresh_time_ > 1000) {
              if (recv_cnt_ > max_fps_) {
                max_fps_ = recv_cnt_;
              }
              recv_cnt_ = 0;
              refresh_time_ = HAL_GetTick();
            }
            ++recv_cnt_;
            amorph::schema::ArmCommandCompressed compressed;
            std::memcpy(&compressed, &data, sizeof(amorph::schema::ArmCommandCompressed));
            arm_cmd_ = amorph::schema::ArmCommand(compressed);
            if (on_arm_cmd_recv_) {
              on_arm_cmd_recv_(arm_cmd_);
            }
          }
          default:
            break;
        }
      }
    });
  }

  void OnJoyRecv(std::function<void(const amorph::schema::Joy &)> cb) { on_joy_recv_ = std::move(cb); }
  void OnArmCommandRecv(std::function<void(const amorph::schema::ArmCommand &)> cb) {
    on_arm_cmd_recv_ = std::move(cb);
  }

  void ResetData() {
    std::memset(&joy_, 0, sizeof(amorph::schema::Joy));
    std::memset(&arm_cmd_, 0, sizeof(amorph::schema::ArmCommand));
  }

  const auto &joy_data() const { return joy_; }
  const auto &arm_cmd_data() const { return arm_cmd_; }

 private:
  std::function<void(const amorph::schema::Joy &)> on_joy_recv_;
  std::function<void(const amorph::schema::ArmCommand &)> on_arm_cmd_recv_;
  amorph::schema::Joy joy_;
  amorph::schema::ArmCommand arm_cmd_{};

  uint32_t refresh_time_{};
  int recv_cnt_{};
  int max_fps_{};
};