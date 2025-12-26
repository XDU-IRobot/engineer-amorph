#pragma once

#include <librm.hpp>

/**
 * @brief 上位机通过CANFD总线转发自定义控制器命令
 */
class CustomController : public rm::device::CanDevice {
 public:
  explicit CustomController(rm::hal::CanInterface &can, uint16_t rx_stdid = 0x233) : CanDevice(can, rx_stdid) {}

  void Write() const { can_->Write(0x233, reinterpret_cast<const uint8_t *>(&data), sizeof(data)); }

  void OnRecv(std::function<void()> cb) { on_recv_cb_ = std::move(cb); }

  struct __attribute__((__packed__)) Command {
    bool ok;
    struct __attribute__((__packed__)) {
      bool a : 1;
      bool b : 1;
      bool c : 1;
      bool d : 1;
      bool e : 1;
      bool f : 1;
      bool g : 1;
      bool h : 1;
    } buttons{};
    float joint_positions[6];
  } data{};

 protected:
  void RxCallback(const rm::hal::CanFrame *msg) override {
    if (msg->dlc == sizeof(data)) {
      ReportStatus(kOk);
      memcpy(&data, msg->data.data(), sizeof(data));
    }
  }

  std::function<void()> on_recv_cb_;
};