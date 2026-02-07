#pragma once

#include <librm.hpp>

struct __attribute__((__packed__)) CustomControllerCommand {
  bool ok;
  struct __attribute__((__packed__)) {
    bool a : 1;
    bool b : 1;
    bool c : 1;
    bool d : 1;
    bool e : 1;
    bool reserved_1 : 1;
    bool reserved_2 : 1;
    bool reserved_3 : 1;
  } buttons{};
  float joint_positions[6];
};

/**
 * @brief 上位机通过CANFD总线转发自定义控制器命令
 * @deprecated 想了一下还是把图传链路串口分出来直接接h7上面，jetson和h7也没离多远
 */
class [[deprecated]] CustomControllerCan : public rm::device::CanDevice {
 public:
  explicit CustomControllerCan(rm::hal::CanInterface &can, uint16_t rx_stdid = 0x233) : CanDevice(can, rx_stdid) {}

  void Write() const { can_->Write(0x233, reinterpret_cast<const uint8_t *>(&data_), sizeof(data_)); }

  void OnRecv(std::function<void()> cb) { on_recv_cb_ = std::move(cb); }

  void RxCallback(const rm::hal::CanFrame *msg) override {
    static CustomControllerCommand temp;
    temp = *reinterpret_cast<const CustomControllerCommand *>(msg->data.data());
    for (int i = 0; i < 6; ++i) {
      if (std::isnan(temp.joint_positions[i]) ||  //
          std::isinf(temp.joint_positions[i]) ||  //
          std::abs(temp.joint_positions[i]) > 20.f) {
        return;  // 数据异常，忽略这一包
      }
    }
    // 数据没问题
    ReportStatus(kOk);
    data_ = temp;
    if (on_recv_cb_) {
      on_recv_cb_();
    }
  }

  CustomControllerCommand data_;

  std::function<void()> on_recv_cb_;
};

template <rm::device::RefereeRevision kRefereeRevision>
class CustomController : public rm::device::Device {
 public:
  explicit CustomController(rm::device::Referee<kRefereeRevision> &ref) : ref_{&ref} {
    ref_->AttachCallback([&](uint16_t cmd_id, uint8_t seq) {
      UNUSED(seq);
      switch (cmd_id) {
        case rm::device::RefereeCmdId<rm::device::RefereeRevision::kV170>::kCustomRobotData: {
          // 把收到的数据转发给状态机处理
          std::memcpy(&data_, ref_->data().custom_robot_data.data, sizeof(data_));
          break;
        }
        default: {
          break;
        }
      }
    });  // 注册解包回调，收到自定义控制器数据时转发命令给状态机
  }

  void OnRecv(std::function<void()> cb) { on_recv_cb_ = std::move(cb); }

 private:
  CustomControllerCommand data_{};
  rm::device::Referee<kRefereeRevision> *ref_;
  std::function<void()> on_recv_cb_{};
};