#pragma once

#include <cstring>

#include <librm.hpp>
#include <etl/circular_buffer.h>
#include <rpc_core.hpp>

#include "rng.h"

class FdCanTransport : public rpc_core::connection {
  class Impl : public rm::device::CanDevice {
    struct StdIdOffset {
      enum {
        kFrameStart = 0,
        kPayload,
        kFrameEnd,
        kHeartbeat,
      };
    };

   public:
    Impl(rm::hal::CanInterface &can, uint16_t rx_stdid_base, uint16_t tx_stdid_base)
        : rm::device::CanDevice(can, rx_stdid_base + StdIdOffset::kFrameStart, rx_stdid_base + StdIdOffset::kPayload,
                                rx_stdid_base + StdIdOffset::kFrameEnd),
          rx_stdid_base_{rx_stdid_base},
          tx_stdid_base_{tx_stdid_base} {}

    /**
     * @brief 发送一个大数据包，该函数会将其分片并通过CAN FD总线发送
     * @param data 要发送的数据指针
     * @param size 要发送的数据大小
     */
    void Send(const uint8_t *data, uint32_t size) {
      // 1. 生成一个随机的帧ID，用于标识本次传输的所有分片
      uint32_t random_frame_id_u32;
      HAL_RNG_GenerateRandomNumber(&hrng, &random_frame_id_u32);
      tx_buf_.frame_start.frame_id = static_cast<uint8_t>(random_frame_id_u32 & 0xff);

      // 2. 准备起始帧，包含元数据：帧ID、总载荷长度和CRC8校验值
      tx_buf_.frame_start.payload_len = size;
      tx_buf_.frame_start.payload_crc8 = rm::modules::Crc8(data, size, rm::modules::CRC8_INIT);
      // 发送起始帧
      can_->Write(tx_stdid_base_ + StdIdOffset::kFrameStart, reinterpret_cast<const uint8_t *>(&tx_buf_.frame_start),
                  sizeof(tx_buf_.frame_start));

      // 3. 循环发送数据分片
      uint16_t fragment_seq = 0;
      uint32_t bytes_sent = 0;
      while (bytes_sent < size) {
        tx_buf_.payload.frame_id = tx_buf_.frame_start.frame_id;
        tx_buf_.payload.fragment_seq = fragment_seq++;
        // 计算当前分片的数据大小
        uint8_t fragment_size =
            std::min(static_cast<uint32_t>(sizeof(tx_buf_.payload.fragment_data)), size - bytes_sent);
        tx_buf_.payload.fragment_len = fragment_size;
        memcpy(tx_buf_.payload.fragment_data, data + bytes_sent, fragment_size);
        // 发送数据分片，DLC固定为64
        can_->Write(tx_stdid_base_ + StdIdOffset::kPayload, reinterpret_cast<const uint8_t *>(&tx_buf_.payload), 64);
        bytes_sent += fragment_size;
      }

      // 4. 发送结束帧，标识数据传输完成
      tx_buf_.frame_end.frame_id = tx_buf_.frame_start.frame_id;
      can_->Write(tx_stdid_base_ + StdIdOffset::kFrameEnd, reinterpret_cast<const uint8_t *>(&tx_buf_.frame_end),
                  sizeof(tx_buf_.frame_end));
    }

    void SendHeartbeat() {
      const uint8_t dummy = 0;
      can_->Write(tx_stdid_base_ + StdIdOffset::kHeartbeat, &dummy, 1);
    }

    void OnFrameRecv(std::function<void(std::string &)> callback) { frame_recv_callback_ = callback; }

   private:
    void RxCallback(const rm::hal::CanMsg *msg) override {
      if (msg->rx_std_id == rx_stdid_base_ + StdIdOffset::kHeartbeat) {
        // 收到心跳帧，说明通信的对方在线
        Heartbeat();
        return;
      }
      // 判断帧类型
      if (msg->rx_std_id == rx_stdid_base_ + StdIdOffset::kFrameStart) {
        // 1. 收到起始帧，开始一个新的数据帧接收会话
        auto *frame_start = reinterpret_cast<const decltype(tx_buf_.frame_start) *>(msg->data.data());
        if (msg->dlc != sizeof(*frame_start)) {
          return;  // 帧长度错误，直接丢弃
        }
        // 初始化接收状态
        rx_state_.frame_id = frame_start->frame_id;
        rx_state_.payload_len = frame_start->payload_len;
        rx_state_.payload_crc8 = frame_start->payload_crc8;
        rx_state_.received_len = 0;
        rx_state_.next_fragment_seq = 0;
        rx_state_.is_receiving = true;
        rx_ring_buffer_.clear();
      } else if (msg->rx_std_id == rx_stdid_base_ + StdIdOffset::kPayload) {
        // 2. 收到数据分片
        if (!rx_state_.is_receiving) {
          return;  // 如果不在接收会话中，则忽略该分片
        }
        auto *payload = reinterpret_cast<const decltype(tx_buf_.payload) *>(msg->data.data());
        // 校验帧ID和分片序号，确保数据的一致性和顺序
        if (payload->frame_id != rx_state_.frame_id) {
          rx_state_.is_receiving = false;  // 帧ID不匹配，放弃接收
          return;
        }
        if (payload->fragment_seq != rx_state_.next_fragment_seq) {
          rx_state_.is_receiving = false;  // 分片序号错误，放弃接收
          return;
        }

        // 将分片数据存入环形缓冲区
        uint32_t data_len = payload->fragment_len;
        if (rx_state_.received_len + data_len > rx_state_.payload_len) {
          rx_state_.is_receiving = false;  // 数据超长，放弃接收
          return;
        }

        for (uint32_t i = 0; i < data_len; ++i) {
          rx_ring_buffer_.push(payload->fragment_data[i]);
        }
        rx_state_.received_len += data_len;
        rx_state_.next_fragment_seq++;

      } else if (msg->rx_std_id == rx_stdid_base_ + StdIdOffset::kFrameEnd) {
        // 3. 收到结束帧
        if (!rx_state_.is_receiving) {
          return;  // 如果不在接收会话中，则忽略
        }
        auto *frame_end = reinterpret_cast<const decltype(tx_buf_.frame_end) *>(msg->data.data());
        if (frame_end->frame_id != rx_state_.frame_id) {
          rx_state_.is_receiving = false;  // 帧ID不匹配，放弃接收
          return;
        }

        // 校验接收到的数据长度是否与起始帧中声明的一致
        if (rx_state_.received_len != rx_state_.payload_len) {
          rx_state_.is_receiving = false;  // 数据长度不匹配，放弃接收
          return;
        }

        // 4. 数据重组和CRC校验
        std::string received_data;
        received_data.resize(rx_state_.payload_len);
        for (uint32_t i = 0; i < rx_state_.payload_len; ++i) {
          received_data[i] = rx_ring_buffer_.front();
          rx_ring_buffer_.pop();
        }

        uint8_t calculated_crc = rm::modules::Crc8(reinterpret_cast<const uint8_t *>(received_data.data()),
                                                   received_data.size(), rm::modules::CRC8_INIT);
        if (calculated_crc == rx_state_.payload_crc8) {
          // 5. CRC校验通过，调用回调函数处理完整的数据帧
          if (frame_recv_callback_) {
            frame_recv_callback_(received_data);
          }
        }
        // 无论成功与否，结束本次接收会话
        rx_state_.is_receiving = false;
      }
    }

    struct {
      struct __attribute__((packed)) {
        uint8_t frame_id;      ///< 这一帧数据的ID，随机生成
        uint16_t payload_len;  ///< 这一帧数据的有效载荷长度，单位字节
        uint8_t payload_crc8;  ///< 这一帧数据的有效载荷的CRC8校验值
      } frame_start;
      struct __attribute__((packed)) {
        uint8_t frame_id;           ///< 这一帧数据的ID，随机生成
        uint16_t fragment_seq;      ///< 这一片段的序号，从0开始
        uint8_t fragment_len;       ///< 这一片段的数据长度
        uint8_t fragment_data[60];  ///< 这一片段的数据
      } payload;
      struct __attribute__((packed)) {
        uint8_t frame_id;  ///< 这一帧数据的ID，随机生成
      } frame_end;

      static_assert(sizeof(frame_start) <= 8);
      static_assert(sizeof(payload) == 64);
      static_assert(sizeof(frame_end) <= 8);
    } tx_buf_;

    struct {
      bool is_receiving = false;
      uint8_t frame_id;
      uint16_t payload_len;
      uint8_t payload_crc8;
      uint16_t received_len;
      uint16_t next_fragment_seq;
    } rx_state_;

    uint16_t rx_stdid_base_;
    uint16_t tx_stdid_base_;
    std::function<void(std::string &)> frame_recv_callback_;  ///< 收到完整帧数据时的回调
    etl::circular_buffer<uint8_t, 4096> rx_ring_buffer_;      ///< 4kb should be enough
  } impl_;

 public:
  FdCanTransport(rm::hal::CanInterface &can, uint16_t rx_stdid_base, uint16_t tx_stdid_base)
      : impl_{can, rx_stdid_base, tx_stdid_base} {
    send_package_impl = [this](const std::string &payload) {
      impl_.Send(reinterpret_cast<const uint8_t *>(payload.data()), payload.size());
    };
    impl_.OnFrameRecv([this](std::string &payload) {
      if (on_recv_package) {
        on_recv_package(payload);
      }
    });
  }

  virtual ~FdCanTransport() = default;

  /**
   * @brief 保证至少每0.5秒调用一次，发送心跳包，让对方知道自己在线
   */
  void SendHeartbeat() { impl_.SendHeartbeat(); }

  bool Connected() { return impl_.IsAlive(); }
};
