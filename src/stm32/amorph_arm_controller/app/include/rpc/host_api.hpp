
#pragma once

#include <rpc_core.hpp>

#include "typedefs.hpp"

namespace host {

extern std::shared_ptr<rpc_core::rpc> rpc;

void Init(const std::shared_ptr<rpc_core::rpc> &rpc_instance);

namespace L {
template <typename... Args>
void Log(LogMessage::LogLevel log_level, const char *fmt, Args &&...args) {
  if (!rpc) {
    return;
  }
  LogMessage msg;
  msg.log_level = log_level;
  msg.str.resize(std::snprintf(nullptr, 0, fmt, std::forward<Args>(args)...) + 1);
  std::snprintf(msg.str.data(), msg.str.size(), fmt, std::forward<Args>(args)...);
  rpc->cmd("log_message")->msg(msg)->call();
}
template <typename... Args>
void D(const char *fmt, Args &&...args) {
  Log(LogMessage::LogLevel::kDebug, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
void I(const char *fmt, Args &&...args) {
  Log(LogMessage::LogLevel::kInfo, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
void W(const char *fmt, Args &&...args) {
  Log(LogMessage::LogLevel::kWarn, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
void E(const char *fmt, Args &&...args) {
  Log(LogMessage::LogLevel::kError, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
void F(const char *fmt, Args &&...args) {
  Log(LogMessage::LogLevel::kFatal, fmt, std::forward<Args>(args)...);
}
};  // namespace L

template <size_t N>
void ReportJointPosition(const JointPosition<N> &jp) {
  rpc->cmd("joint_position")->msg(jp)->call();
}
template <size_t N>
void ReportJointPosition(const JointPosition<N> &&jp) {
  rpc->cmd("joint_position")->msg(std::forward<const JointPosition<N>>(jp))->call();
}

}  // namespace host