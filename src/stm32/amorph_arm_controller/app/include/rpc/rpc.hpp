#pragma once

#include <rpc_core.hpp>

#include "fdcan_transport.hpp"
#include "polling_timer.hpp"

/**
 * @brief   基于rpc_core的RPC通信类
 * @note    该类封装了rpc_core库，提供了更高层次的RPC通信接口，简化了与上位机的交互。
 */
class RPC {
 public:
  RPC() = default;
  explicit RPC(std::shared_ptr<FdCanTransport> &conn) : rpc_{rpc_core::rpc::create(conn)}, conn_{conn} {
    rpc_->set_timer([this](uint32_t delay_ms, std::function<void()> cb) {
      timer_.DeferredCall(cb, std::chrono::milliseconds(delay_ms));
      // 在linux上位机上实现时可以直接用std::async，轮询调度只是一种裸机环境下的替代方案
    });
  }

  /**
   * @brief 绑定一个已经创建好的连接
   * @param conn
   */
  void AttachToConnection(std::shared_ptr<FdCanTransport> &conn) {
    conn_ = conn;
    rpc_ = rpc_core::rpc::create(conn_);
  }

  /**
   * @brief 至少每20ms调用一次，以维持连接活跃
   */
  void Poll() {
    timer_.Poll();
    conn_->Poll();
    rpc_->set_ready(conn_->Connected());
  }

  /**
   * @return RPC连接是否已建立
   */
  bool Connected() const { return conn_->Connected(); }

  auto rpc() { return rpc_; }
  auto conn() { return conn_; }

 private:
  std::shared_ptr<rpc_core::rpc> rpc_{nullptr};
  std::shared_ptr<FdCanTransport> conn_{nullptr};
  PollingTimer timer_;  ///< 轮询定时器
};