
#pragma once

#include <format>

#include <rpc_core.hpp>

#include "fdcan_transport.hpp"
#include "typedefs.hpp"
#include "globals.hpp"
#include "polling_timer.hpp"

class RPC {
 public:
  RPC() = default;
  explicit RPC(std::shared_ptr<FdCanTransport> &conn) : rpc_{rpc_core::rpc::create(conn)}, conn_{conn} {
    rpc_->set_timer([this](uint32_t delay_ms, std::function<void()> cb) {
      scheduler_.DeferredCall(cb, std::chrono::milliseconds(delay_ms));
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
   * @brief 心跳保持函数，至少每0.5秒调用一次，以维持连接活跃
   */
  void KeepConnection() {
    conn_->SendHeartbeat();
    rpc_->set_ready(conn_->Connected());
  }

  /**
   * @brief 定时轮询函数，至少每10ms调用一次，以处理RPC后台任务
   */
  void TickTimer() { scheduler_.Poll(); }

  /**
   * @return RPC连接是否已建立
   */
  bool Connected() { return conn_->Connected(); }

  auto rpc() { return rpc_; }

 private:
  std::shared_ptr<rpc_core::rpc> rpc_{nullptr};
  std::shared_ptr<FdCanTransport> conn_{nullptr};
  PollingTimer scheduler_;  ///< 轮询调度器

  // remote functions
 public:
  template <typename... Args>
  void Debug(const std::string &fmt, Args &&...args) {
    const auto msg = [&] {
      LogMessage msg;
      msg.log_level = LogMessage::LogLevel::kDebug;
      msg.str = std::format(fmt, std::forward<Args>(args)...);
      return msg;
    }();
    rpc_->cmd("log_message")->msg(msg)->call();
  }

  template <typename... Args>
  void Info(const std::string &fmt, Args &&...args) {
    const auto msg = [&] {
      LogMessage msg;
      msg.log_level = LogMessage::LogLevel::kInfo;
      msg.str = std::format(fmt, std::forward<Args>(args)...);
      return msg;
    }();
    rpc_->cmd("log_message")->msg(msg)->call();
  }

  template <typename... Args>
  void Warn(const std::string &fmt, Args &&...args) {
    const auto msg = [&] {
      LogMessage msg;
      msg.log_level = LogMessage::LogLevel::kWarn;
      msg.str = std::format(fmt, std::forward<Args>(args)...);
      return msg;
    }();
    rpc_->cmd("log_message")->msg(msg)->call();
  }

  template <typename... Args>
  void Error(const std::string &fmt, Args &&...args) {
    const auto msg = [&] {
      LogMessage msg;
      msg.log_level = LogMessage::LogLevel::kError;
      msg.str = std::format(fmt, std::forward<Args>(args)...);
      return msg;
    }();
    rpc_->cmd("log_message")->msg(msg)->call();
  }

  template <typename... Args>
  [[noreturn]] void Fatal(const std::string &fmt, Args &&...args) {
    const auto msg = [&] {
      LogMessage msg;
      msg.log_level = LogMessage::LogLevel::kFatal;
      msg.str = std::format(fmt, std::forward<Args>(args)...);
      return msg;
    }();
    rpc_->cmd("log_message")->msg(msg)->call();
    while (true) {
    }
  }

  void ReportJointPosition(const JointPosition &jp) { rpc_->cmd("joint_position")->msg(jp)->call(); }
};