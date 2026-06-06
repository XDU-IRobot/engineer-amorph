#pragma once

#include <librm.hpp>

#include "custom_controller_schema.hpp"

/// @brief 两用工具类，从机调用ReportToMaster向主机反馈数据，主机调用MasterStartRx接收从机反馈
/// @note  接收端使用逐字节状态机（参考 Referee 实现），CRC16 校验通过后才提交帧
class MasterSlaveCan : public rm::device::CanDevice {
  constexpr static uint16_t kBaseFrameId = 0x100;

 public:
  explicit MasterSlaveCan(rm::hal::CanInterface &can)
      : CanDevice{can, kBaseFrameId, kBaseFrameId + 1, kBaseFrameId + 2, kBaseFrameId + 3} {}

  /// @brief 从机端：填充 CRC 并阻塞发送一帧s
  void ReportToMaster(const SlaveData &data) {
    switch (tx_seq_) {
      case 0: {
        tx_frag_1.ok = data.ok;
        can_->Write(kBaseFrameId, (uint8_t *)&tx_frag_1, sizeof(tx_frag_1));
        break;
      }
      case 1: {
        tx_frag_2.q_12[0] = data.q[0];
        tx_frag_2.q_12[1] = data.q[1];
        can_->Write(kBaseFrameId + 1, (uint8_t *)&tx_frag_2, sizeof(tx_frag_2));
        break;
      }
      case 2: {
        tx_frag_3.q_34[0] = data.q[3];
        tx_frag_3.q_34[1] = data.q[4];
        can_->Write(kBaseFrameId + 2, (uint8_t *)&tx_frag_3, sizeof(tx_frag_3));
        break;
      }
      case 3: {
        tx_frag_4.q_56[0] = data.q[4];
        tx_frag_4.q_56[1] = data.q[5];
        can_->Write(kBaseFrameId + 4, (uint8_t *)&tx_frag_3, sizeof(tx_frag_3));
        break;
      }
    }
    tx_seq_ = (tx_seq_ + 1) % 4;
  }

  const auto &data() const { return data_; }

 private:
  void RxCallback(const rm::hal::CanFrame *msg) override {
    ReportStatus(kOk);
    switch (msg->rx_std_id - kBaseFrameId) {
      case 0: {
        std::memcpy(&rx_frag_1, msg->data.data(), sizeof(SlaveDataCan1));
        break;
      }
      case 1: {
        std::memcpy(&rx_frag_2, msg->data.data(), sizeof(SlaveDataCan2));
        break;
      }
      case 2: {
        std::memcpy(&rx_frag_3, msg->data.data(), sizeof(SlaveDataCan3));
        break;
      }
      case 3: {
        std::memcpy(&rx_frag_4, msg->data.data(), sizeof(SlaveDataCan4));
        break;
      }
      default:
        break;
    }
    data_ = SlaveData{rx_frag_1, rx_frag_2, rx_frag_3, rx_frag_4};
  }

  SlaveDataCan1 tx_frag_1{};
  SlaveDataCan2 tx_frag_2{};
  SlaveDataCan3 tx_frag_3{};
  SlaveDataCan4 tx_frag_4{};

  SlaveDataCan1 rx_frag_1{};
  SlaveDataCan2 rx_frag_2{};
  SlaveDataCan3 rx_frag_3{};
  SlaveDataCan4 rx_frag_4{};

  uint8_t tx_seq_{0};

  SlaveData data_{};
};