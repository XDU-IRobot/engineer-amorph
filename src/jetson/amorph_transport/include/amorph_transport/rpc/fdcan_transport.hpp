#pragma once

#include <cstring>
#include <chrono>

#include <librm.hpp>
#include <etl/circular_buffer.h>
#include <etl/unordered_map.h>
#include <etl/vector.h>
#include <rpc_core.hpp>

#include "logger.hpp"

namespace amorph {

/**
 * @brief 基于CAN FD实现的一种rpc_core框架的传输层（带ACK确认和重传机制）
 *
 * 改进点：
 * 1. 添加ACK/NACK确认机制，接收方收到完整帧后回复ACK
 * 2. 发送方通过轮询检测ACK，超时自动重传
 * 3. 可配置重试次数和超时时间
 * 4. 序列号机制防止重复处理
 * 5. 完全基于轮询，无操作系统依赖，适用于裸机环境
 */
class FdCanTransport : public rpc_core::connection {
  /**
   * @brief 基于CAN FD总线的大数据传输类（可靠版本，裸机兼容）
   *
   * 该类实现了一个基于CAN FD总线的大数据传输协议，支持将大数据包分片发送和接收。
   * 通过ACK确认和超时重传机制，实现了可靠的数据传输。
   * 完全基于轮询实现，不依赖操作系统原语。
   */
  class Impl : public rm::device::CanDevice {
    struct StdIdOffset {
      enum {
        kFrameStart = 0,
        kFrameEnd,
        kPayload,
        kAck,        ///< ACK确认帧
        kNack,       ///< NACK否定确认帧（用于快速重传）
        kHeartbeat = 40,  // 给一个很低的优先级，防止数据帧被同时发送的心跳仲裁掉
      };
    };

   public:
    static constexpr uint32_t kMaxPayloadBytes = 4096;
    static constexpr uint32_t kHeartbeatIntervalMs = 200;  ///< 心跳间隔，小于超时时间的一半
    static constexpr uint32_t kAckTimeoutMs = 50;          ///< ACK等待超时时间
    static constexpr uint32_t kMaxRetries = 5;             ///< 最大重试次数
    static constexpr uint32_t kMaxPendingTx = 4;           ///< 最大待确认发送队列长度

    using Clock = std::chrono::steady_clock;

    Impl(rm::hal::CanInterface &can, uint16_t rx_stdid_base, uint16_t tx_stdid_base)
        : rm::device::CanDevice(can, rx_stdid_base + StdIdOffset::kFrameStart, rx_stdid_base + StdIdOffset::kFrameEnd,
                                rx_stdid_base + StdIdOffset::kPayload, rx_stdid_base + StdIdOffset::kAck,
                                rx_stdid_base + StdIdOffset::kNack, rx_stdid_base + StdIdOffset::kHeartbeat),
          rx_stdid_base_{rx_stdid_base},
          tx_stdid_base_{tx_stdid_base},
          last_tx_time_{Clock::now()} {}

    /**
     * @brief 发送一个大数据包（非阻塞，加入发送队列）
     * @param data 要发送的数据指针
     * @param size 要发送的数据大小
     * @return true 成功加入发送队列，false 队列已满
     *
     * @note 发送不是立即完成的，需要周期性调用 Poll() 来处理ACK和重传
     */
    bool Send(const uint8_t *data, uint32_t size) {
      if (size > kMaxPayloadBytes) {
        L::E(text::Red("Payload size %u exceeds maximum %u"), size, kMaxPayloadBytes);
        return false;
      }

      if (pending_tx_queue_.full()) {
        L::E(text::Red("TX queue full, dropping packet"));
        return false;
      }

      // 创建新的待发送条目
      PendingTx tx_entry;
      tx_entry.data.resize(size);
      memcpy(tx_entry.data.data(), data, size);
      tx_entry.retry_count = 0;
      tx_entry.frame_id = GenerateFrameId();
      tx_entry.state = TxState::kPendingSend;

      pending_tx_queue_.push_back(std::move(tx_entry));
      return true;
    }

    /**
     * @brief 轮询处理函数，必须周期性调用（建议10-20ms）
     *
     * 此函数处理：
     * 1. 发送队列中待发送的数据
     * 2. 检查ACK超时并触发重传
     * 3. 清理已确认的发送条目
     * 4. 智能心跳发送
     */
    void Poll() {
      auto now = Clock::now();

      // 处理发送队列
      for (auto it = pending_tx_queue_.begin(); it != pending_tx_queue_.end();) {
        auto &tx = *it;

        switch (tx.state) {
          case TxState::kPendingSend:
            // 执行实际发送
            DoTransmit(tx);
            tx.state = TxState::kWaitingAck;
            tx.last_send_time = now;
            ++it;
            break;

          case TxState::kWaitingAck: {
            // 检查是否收到ACK
            if (tx.ack_received) {
              // 发送成功，移除条目
              it = pending_tx_queue_.erase(it);
              continue;
            }

            // 检查超时
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - tx.last_send_time).count();
            if (elapsed_ms >= kAckTimeoutMs) {
              if (tx.retry_count >= kMaxRetries) {
                // 超过最大重试次数，放弃
                L::E(text::Red("Failed to send frame %u after %u retries"), tx.frame_id, kMaxRetries);
                it = pending_tx_queue_.erase(it);
                continue;
              }
              // 重传
              tx.retry_count++;
              L::W(text::Yellow("Retrying frame %u, attempt %u/%u"), tx.frame_id, tx.retry_count, kMaxRetries);
              // 生成新的frame_id避免与旧ACK混淆
              tx.frame_id = GenerateFrameId();
              tx.ack_received = false;
              DoTransmit(tx);
              tx.last_send_time = now;
            }
            ++it;
            break;
          }

          default:
            ++it;
            break;
        }
      }

      // 智能心跳：仅在无数据传输时发送
      SendHeartbeatIfNeeded();
    }

    /**
     * @brief 检查发送队列是否为空（所有数据都已确认）
     */
    bool IsTxQueueEmpty() const { return pending_tx_queue_.empty(); }

    /**
     * @brief 智能心跳发送：仅在长时间无数据传输时发送心跳
     */
    void SendHeartbeatIfNeeded() {
      auto now = Clock::now();
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tx_time_).count();
      if (elapsed_ms >= kHeartbeatIntervalMs) {
        can_->Write(tx_stdid_base_ + StdIdOffset::kHeartbeat, nullptr, 0);
        last_tx_time_ = now;
      }
    }

    void OnFrameRecv(std::function<void(std::string &)> callback) { frame_recv_callback_ = std::move(callback); }

   private:
    // ===== 类型前向声明（供成员函数使用）=====

    enum class TxState {
      kPendingSend,  ///< 等待发送
      kWaitingAck,   ///< 已发送，等待ACK
    };

    struct PendingTx {
      etl::vector<uint8_t, kMaxPayloadBytes> data;  ///< 待发送数据
      uint8_t frame_id{0};                          ///< 帧ID
      uint8_t retry_count{0};                       ///< 已重试次数
      TxState state{TxState::kPendingSend};         ///< 发送状态
      Clock::time_point last_send_time;             ///< 上次发送时间
      volatile bool ack_received{false};            ///< ACK接收标志
    };

    /**
     * @brief 生成随机帧ID
     */
    static uint8_t GenerateFrameId() {
      // 简单的线性同余生成器，适用于裸机环境
      static uint32_t seed = 0x12345678;
      seed = seed * 1103515245 + 12345;
      return static_cast<uint8_t>((seed >> 16) & 0xFF);
    }

    /**
     * @brief 执行实际的数据发送（分片传输）
     */
    void DoTransmit(PendingTx &tx) {
      last_tx_time_ = Clock::now();

      const uint8_t *data = tx.data.data();
      uint32_t size = tx.data.size();

      // 1. 发送起始帧
      tx_buf_.frame_start.frame_id = tx.frame_id;
      tx_buf_.frame_start.payload_len = size;
      tx_buf_.frame_start.payload_crc8 = rm::modules::Crc8(data, size, rm::modules::CRC8_INIT);
      can_->Write(tx_stdid_base_ + StdIdOffset::kFrameStart, reinterpret_cast<const uint8_t *>(&tx_buf_.frame_start),
                  sizeof(tx_buf_.frame_start));

      // 2. 发送数据分片
      uint16_t fragment_seq = 0;
      uint32_t bytes_sent = 0;
      while (bytes_sent < size) {
        tx_buf_.payload.frame_id = tx.frame_id;
        tx_buf_.payload.fragment_seq = fragment_seq++;
        uint8_t fragment_size =
            std::min(static_cast<uint32_t>(sizeof(tx_buf_.payload.fragment_data)), size - bytes_sent);
        tx_buf_.payload.fragment_len = fragment_size;
        memcpy(tx_buf_.payload.fragment_data, data + bytes_sent, fragment_size);
        can_->Write(tx_stdid_base_ + StdIdOffset::kPayload, reinterpret_cast<const uint8_t *>(&tx_buf_.payload), 64);
        bytes_sent += fragment_size;
      }

      // 3. 发送结束帧
      tx_buf_.frame_end.frame_id = tx.frame_id;
      can_->Write(tx_stdid_base_ + StdIdOffset::kFrameEnd, reinterpret_cast<const uint8_t *>(&tx_buf_.frame_end),
                  sizeof(tx_buf_.frame_end));
    }

    void RxCallback(const rm::hal::CanFrame *msg) override {
      // 收到任何帧都更新连接状态
      ReportStatus(kOk);

      // 判断帧类型
      if (msg->rx_std_id == rx_stdid_base_ + StdIdOffset::kHeartbeat) {
        // 心跳帧，仅用于保持连接状态
        return;
      }

      if (msg->rx_std_id == rx_stdid_base_ + StdIdOffset::kAck) {
        // 收到ACK确认帧
        const auto *ack = reinterpret_cast<const AckFrame *>(msg->data.data());
        // 在发送队列中查找对应的frame_id并标记为已确认
        for (auto &tx : pending_tx_queue_) {
          if (tx.frame_id == ack->frame_id && tx.state == TxState::kWaitingAck) {
            tx.ack_received = true;
            break;
          }
        }
        return;
      }

      if (msg->rx_std_id == rx_stdid_base_ + StdIdOffset::kNack) {
        // 收到NACK，标记需要立即重传
        const auto *nack = reinterpret_cast<const AckFrame *>(msg->data.data());
        for (auto &tx : pending_tx_queue_) {
          if (tx.frame_id == nack->frame_id && tx.state == TxState::kWaitingAck) {
            // 通过将last_send_time设为过去来触发立即重传
            tx.last_send_time = Clock::now() - std::chrono::milliseconds(kAckTimeoutMs + 1);
            break;
          }
        }
        return;
      }

      if (msg->rx_std_id == rx_stdid_base_ + StdIdOffset::kFrameStart) {
        // 收到起始帧，开始一个新的数据帧接收会话
        const auto *frame_start = reinterpret_cast<const decltype(tx_buf_.frame_start) *>(msg->data.data());
        if (msg->dlc != sizeof(*frame_start)) {
          return;  // 帧长度错误
        }
        if (frame_start->payload_len > kMaxPayloadBytes) {
          rx_sessions_.erase(frame_start->frame_id);
          return;
        }
        // 检查会话池是否已满
        if (rx_sessions_.find(frame_start->frame_id) == rx_sessions_.end() && rx_sessions_.full()) {
          EvictLeastRecentlyUsedSession();
        }
        auto &session = rx_sessions_[frame_start->frame_id];
        session.payload_len = frame_start->payload_len;
        session.payload_crc8 = frame_start->payload_crc8;
        session.next_fragment_seq = 0;
        session.last_active_time = Clock::now();
        session.buffer.clear();
        session.buffer.reserve(frame_start->payload_len);
        return;
      }

      if (msg->rx_std_id == rx_stdid_base_ + StdIdOffset::kPayload) {
        // 收到数据分片
        const auto *payload = reinterpret_cast<const decltype(tx_buf_.payload) *>(msg->data.data());
        auto session_it = rx_sessions_.find(payload->frame_id);
        if (session_it == rx_sessions_.end()) {
          return;  // 起始帧缺失
        }
        auto &session = session_it->second;
        if (payload->fragment_seq != session.next_fragment_seq) {
          rx_sessions_.erase(session_it);  // 顺序错误
          return;
        }
        const uint32_t data_len = payload->fragment_len;
        if (session.buffer.size() + data_len > session.payload_len) {
          rx_sessions_.erase(session_it);  // 数据超长
          return;
        }
        session.buffer.insert(session.buffer.end(), payload->fragment_data, payload->fragment_data + data_len);
        session.next_fragment_seq++;
        session.last_active_time = Clock::now();
        return;
      }

      if (msg->rx_std_id == rx_stdid_base_ + StdIdOffset::kFrameEnd) {
        // 收到结束帧
        const auto *frame_end = reinterpret_cast<const decltype(tx_buf_.frame_end) *>(msg->data.data());
        auto session_it = rx_sessions_.find(frame_end->frame_id);
        if (session_it == rx_sessions_.end()) {
          return;
        }
        auto &session = session_it->second;
        if (session.buffer.size() != session.payload_len) {
          rx_sessions_.erase(session_it);  // 长度不一致
          return;
        }

        // CRC校验
        uint8_t calculated_crc = rm::modules::Crc8(session.buffer.empty() ? nullptr : session.buffer.data(),
                                                   session.buffer.size(), rm::modules::CRC8_INIT);
        if (calculated_crc == session.payload_crc8) {
          // 检查是否是重复帧（去重）
          if (frame_end->frame_id == last_received_frame_id_) {
            // 重复帧，发送ACK但不处理数据
            SendAck(frame_end->frame_id);
            rx_sessions_.erase(session_it);
            return;
          }
          last_received_frame_id_ = frame_end->frame_id;

          // CRC校验通过，发送ACK
          SendAck(frame_end->frame_id);

          // 调用回调函数处理数据
          if (frame_recv_callback_) {
            std::string received_data(session.buffer.begin(), session.buffer.end());
            frame_recv_callback_(received_data);
          }
        } else {
          // CRC校验失败，发送NACK
          SendNack(frame_end->frame_id);
        }
        rx_sessions_.erase(session_it);
      }
    }

    void SendAck(uint8_t frame_id) {
      AckFrame ack = {frame_id};
      can_->Write(tx_stdid_base_ + StdIdOffset::kAck, reinterpret_cast<const uint8_t *>(&ack), sizeof(ack));
    }

    void SendNack(uint8_t frame_id) {
      AckFrame nack = {frame_id};
      can_->Write(tx_stdid_base_ + StdIdOffset::kNack, reinterpret_cast<const uint8_t *>(&nack), sizeof(nack));
    }

    // ===== 数据结构定义 =====

    struct __attribute__((packed)) AckFrame {
      uint8_t frame_id;
    };
    static_assert(sizeof(AckFrame) == 1);

    struct {
      struct __attribute__((packed)) {
        uint8_t frame_id;
        uint16_t payload_len;
        uint8_t payload_crc8;
      } frame_start;
      struct __attribute__((packed)) {
        uint8_t frame_id;
        uint16_t fragment_seq;
        uint8_t fragment_len;
        uint8_t fragment_data[60];
      } payload;
      struct __attribute__((packed)) {
        uint8_t frame_id;
      } frame_end;

      static_assert(sizeof(frame_start) == 4);
      static_assert(sizeof(payload) == 64);
      static_assert(sizeof(frame_end) == 1);
    } tx_buf_;

    etl::vector<PendingTx, kMaxPendingTx> pending_tx_queue_;  ///< 待确认发送队列

    struct RxSession {
      uint16_t payload_len = 0;
      uint8_t payload_crc8 = 0;
      uint16_t next_fragment_seq = 0;
      Clock::time_point last_active_time;
      etl::vector<uint8_t, kMaxPayloadBytes> buffer;
    };

    etl::unordered_map<uint8_t, RxSession, 5> rx_sessions_;

    uint16_t rx_stdid_base_;
    uint16_t tx_stdid_base_;
    Clock::time_point last_tx_time_;
    std::function<void(std::string &)> frame_recv_callback_;
    uint8_t last_received_frame_id_{0};

    void EvictLeastRecentlyUsedSession() {
      if (rx_sessions_.empty()) return;

      L::W(text::Yellow("Evicting least recently used RX session due to capacity limit."));

      auto oldest_it = rx_sessions_.begin();
      auto oldest_time = oldest_it->second.last_active_time;

      for (auto it = rx_sessions_.begin(); it != rx_sessions_.end(); ++it) {
        if (it->second.last_active_time < oldest_time) {
          oldest_time = it->second.last_active_time;
          oldest_it = it;
        }
      }

      rx_sessions_.erase(oldest_it);
    }
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
   * @brief 轮询处理函数，必须周期性调用
   *
   * 此函数处理发送队列、ACK超时重传和心跳发送。
   * 建议调用周期：10-20ms
   */
  void Poll() { impl_.Poll(); }

  /**
   * @brief 检查连接状态
   */
  bool Connected() { return impl_.online_status() == rm::device::Device::kOk; }

  /**
   * @brief 检查发送队列是否为空
   */
  bool IsTxQueueEmpty() const { return impl_.IsTxQueueEmpty(); }
};

}  // namespace amorph
