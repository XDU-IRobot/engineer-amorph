
#pragma once

#include <random>
#include <cstring>

#include <etl/circular_buffer.h>
#include <rpc_core.hpp>

#include "usbd_cdc_if.h"
#include "crc.h"

namespace rpc {

class UsbcdcTransport : public rpc_core::connection {
  enum class ReceiveState { kIdle, kPayloadLen, kReceivingPayload, kCrc8 } rx_state_;
  const std::string frame_start_ = "amorph";
  struct {
    uint8_t payload_len_remaining;
    uint32_t this_frame_len;
  } rx_auxiliary_;

  void Send(const uint8_t *data, uint32_t size) {
    uint8_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)data, size);
    CDC_Transmit_HS((uint8_t *)frame_start_.data(), frame_start_.size());
    CDC_Transmit_HS(reinterpret_cast<uint8_t *>(&size), sizeof(size));
    CDC_Transmit_HS(const_cast<uint8_t *>(data), size);
    CDC_Transmit_HS(&crc, 1);
  }

  void RxCallback(uint8_t byte) {
    rx_ring_buffer_.push(byte);
    switch (rx_state_) {
      case ReceiveState::kIdle: {
        if (std::equal(frame_start_.begin(), frame_start_.end(), rx_ring_buffer_.begin())) {
          rx_auxiliary_.payload_len_remaining = 4;
          rx_state_ = ReceiveState::kPayloadLen;
        }
        break;
      }
      case ReceiveState::kPayloadLen: {
        rx_auxiliary_.payload_len_remaining--;
        if (rx_auxiliary_.payload_len_remaining == 0) {
          rx_auxiliary_.this_frame_len = (rx_ring_buffer_[0]) | (rx_ring_buffer_[1] << 8) | (rx_ring_buffer_[2] << 16) |
                                         (rx_ring_buffer_[3] << 24);
          rx_state_ = ReceiveState::kReceivingPayload;
        }
        break;
      }
      case ReceiveState::kReceivingPayload: {
        rx_auxiliary_.this_frame_len--;
        if (rx_auxiliary_.this_frame_len == 0) {
          rx_state_ = ReceiveState::kCrc8;
        }
        break;
      }
      case ReceiveState::kCrc8: {
        rx_state_ = ReceiveState::kIdle;
        if (HAL_CRC_Calculate(&hcrc, (uint32_t *)rx_ring_buffer_[1], rx_auxiliary_.this_frame_len) == byte) {
          on_recv_package(
              std::string(rx_ring_buffer_.begin() + 1, rx_ring_buffer_.begin() + 1 + rx_auxiliary_.this_frame_len));
        }
        break;
      }
    }
  }

  etl::circular_buffer<uint8_t, 4096> rx_ring_buffer_;  ///< 4kb should be enough

 public:
  UsbcdcTransport() {
    send_package_impl = [this](const std::string &payload) {
      Send(reinterpret_cast<const uint8_t *>(payload.data()), payload.size());
    };
  }

  virtual ~UsbcdcTransport() = default;
};

}  // namespace rpc
