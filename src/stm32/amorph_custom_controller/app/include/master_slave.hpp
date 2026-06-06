#pragma once

#include <librm.hpp>

#include "custom_controller_schema.hpp"

/// @brief 两用工具类，从机调用ReportToMaster向主机反馈数据，主机调用MasterStartRx接收从机反馈
/// @note  接收端使用逐字节状态机（参考 Referee 实现），CRC16 校验通过后才提交帧
class MasterSlave : public rm::device::Device {
 public:
  explicit MasterSlave(UART_HandleTypeDef &huart = huart3) : ser_{huart, true, true} {}

  /// @brief 从机端：填充 CRC 并阻塞发送一帧s
  void ReportToMaster(const SlaveData &data) {
    data_ = data;
    data_.crc16 = rm::modules::Crc16(reinterpret_cast<const uint8_t *>(&data_), sizeof(SlaveData) - sizeof(uint16_t),
                                     rm::modules::CRC16_INIT);
    ser_.WriteAsync(reinterpret_cast<const uint8_t *>(&data_), sizeof(data_), [] {});
  }

  /// @brief 主机端：启动持续接收
  void MasterStartRx() {
    ser_.AttachRxCallback([this](etl::span<const uint8_t> chunk) {
      for (size_t i = 0; i < chunk.size(); ++i) {
        ++bytes_received_;
        FeedByte(chunk[i]);
      }
    });
    ser_.Start();
  }

  const auto &data() const { return data_; }

 private:
  enum class RxState {
    kSof1,     ///< 等待第一个 0xa5
    kSof2,     ///< 等待第二个 0xa5
    kPayload,  ///< 累积负载数据
  };

  static constexpr size_t kFrameSize = sizeof(SlaveData);

  /// @brief 逐字节状态机，任何字节不符合预期立即复位
  void FeedByte(uint8_t byte) {
    switch (state_) {
      case RxState::kSof1: {
        if (byte == 0xa5) {
          buf_[0] = byte;
          buf_idx_ = 1;
          state_ = RxState::kSof2;
        }
        // else: 丢字节，继续等
        break;
      }

      case RxState::kSof2: {
        if (byte == 0xa5) {
          buf_[1] = byte;
          buf_idx_ = 2;
          state_ = RxState::kPayload;
        } else {
          state_ = RxState::kSof1;  // 第二个字节不是 0xa5，从头开始
        }
        break;
      }

      case RxState::kPayload: {
        buf_[buf_idx_++] = byte;
        if (buf_idx_ >= kFrameSize) {
          state_ = RxState::kSof1;
          buf_idx_ = 0;

          uint16_t rx_crc;
          std::memcpy(&rx_crc, buf_ + kFrameSize - sizeof(uint16_t), sizeof(uint16_t));
          uint16_t calc_crc = rm::modules::Crc16(buf_, kFrameSize - sizeof(uint16_t), rm::modules::CRC16_INIT);

          if (rx_crc == calc_crc) {
            ReportStatus(kOk);
            std::memcpy(&data_, buf_, kFrameSize);
          }
          // CRC 失败：静默丢弃，回到 kSof1 等下一帧
        }
        break;
      }
    }
  }

  size_t bytes_received_{0};
  RxState state_{RxState::kSof1};
  size_t buf_idx_{0};
  uint8_t buf_[kFrameSize]{};
  SlaveData data_{};
  rm::hal::Serial<sizeof(SlaveData)> ser_;
};