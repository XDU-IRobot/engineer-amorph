#pragma once

#include <thread>

#include <rpc_core.hpp>

#include "fdcan_transport.hpp"

namespace amorph {

/**
 * @brief   基于rpc_core的RPC通信类
 * @note    该类封装了rpc_core库，提供了更高层次的RPC通信接口，简化了与上位机的交互。
 */
class RPC {
 public:
  RPC() = default;
  explicit RPC(std::shared_ptr<FdCanTransport> &conn) : rpc_{rpc_core::rpc::create(conn)}, conn_{conn} {
    rpc_->set_timer([this](uint32_t delay_ms, const std::function<void()> &cb) {
      std::thread([delay_ms, cb]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        cb();
      }).detach();
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
   * @brief 心跳保持函数，至少每0.5秒调用一次，以维持连接活跃
   */
  void KeepConnection() const {
    conn_->SendHeartbeat();
    rpc_->set_ready(conn_->Connected());
    // rpc_->set_ready(true);
  }

  /**
   * @return RPC连接是否已建立
   */
  bool Connected() const { return conn_->Connected(); }

  auto rpc() { return rpc_; }

 private:
  std::shared_ptr<rpc_core::rpc> rpc_{nullptr};
  std::shared_ptr<FdCanTransport> conn_{nullptr};
};

}  // namespace amorph