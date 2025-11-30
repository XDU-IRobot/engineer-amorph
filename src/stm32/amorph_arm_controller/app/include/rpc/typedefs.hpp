
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <rpc_core.hpp>

/**
 * @note    slave -> host only
 */
struct LogMessage {
  enum LogLevel : uint8_t { kDebug = 0, kInfo = 1, kWarn = 2, kError = 3, kFatal = 4 };
  LogLevel log_level{LogLevel::kInfo};
  std::string str;
  RPC_CORE_DEFINE_TYPE_INNER(log_level, str);
};

/**
 * @brief   用于向上位机反馈当前关节位置，因为需要高频传输，所以只传位置，节省带宽
 * @note    slave -> host only
 */
struct JointPosition {
  std::vector<float> positions;
  RPC_CORE_DEFINE_TYPE_INNER(positions);
};

struct JointTrajectoryPoint {
  std::vector<float> positions;
  std::vector<float> velocities;
  std::vector<float> accelerations;
  std::vector<float> effort;
  std::chrono::duration<int, std::milli> time_from_start;
  RPC_CORE_DEFINE_TYPE_INNER(positions, velocities, accelerations, effort, time_from_start);
};

struct JointTrajectory {
  std::vector<JointTrajectoryPoint> points;
  RPC_CORE_DEFINE_TYPE_INNER(points);
};

/**
 * @note    host -> slave only
 */
struct SbusFrame {
  std::vector<float> ch;
  RPC_CORE_DEFINE_TYPE_INNER(ch);
};
