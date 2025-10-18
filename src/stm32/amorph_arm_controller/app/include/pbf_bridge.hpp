#pragma once

#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <cstdint>

#include "nanopb_cpp.h"
#include "host_slave.pb.h"

namespace proto {

enum class LogLevel { kDebug = 0, kInfo = 1, kWarn = 2, kError = 3, kFatal = 4 };

struct LogMessage {
  LogLevel log_level{LogLevel::kInfo};
  std::string str;
};

struct Duration {
  int64_t seconds = 0;
  int32_t nanoseconds = 0;
};

struct JointTrajectoryPoint {
  std::vector<float> positions;
  std::vector<float> velocities;
  std::vector<float> accelerations;
  std::vector<float> effort;
  Duration time_from_start;
};

struct JointTrajectory {
  std::vector<JointTrajectoryPoint> points;
};

struct SbusFrame {
  std::vector<float> ch;
};

/**
 * PbfBridge - 提供简洁的序列化和反序列化接口
 * 使用基础的nanopb API，避免复杂的callback机制
 */
class PbfBridge {
 private:
  // 字符串编码回调
  static bool encode_string_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg) {
    const std::string* str = static_cast<const std::string*>(*arg);
    if (!pb_encode_tag_for_field(stream, field)) {
      return false;
    }
    return pb_encode_string(stream, reinterpret_cast<const pb_byte_t*>(str->c_str()), str->size());
  }

  // 字符串解码回调
  static bool decode_string_callback(pb_istream_t* stream, const pb_field_t* field, void** arg) {
    std::string* str = static_cast<std::string*>(*arg);

    pb_byte_t buffer[1024];
    size_t len = stream->bytes_left;

    if (len > sizeof(buffer) - 1) {
      len = sizeof(buffer) - 1;
    }

    if (!pb_read(stream, buffer, len)) {
      return false;
    }

    buffer[len] = '\0';
    *str = reinterpret_cast<char*>(buffer);
    return true;
  }

  // 浮点数数组编码回调
  static bool encode_float_array_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg) {
    const std::vector<float>* vec = static_cast<const std::vector<float>*>(*arg);

    for (float value : *vec) {
      if (!pb_encode_tag_for_field(stream, field)) {
        return false;
      }
      if (!pb_encode_float(stream, &value)) {
        return false;
      }
    }
    return true;
  }

  // 浮点数数组解码回调
  static bool decode_float_array_callback(pb_istream_t* stream, const pb_field_t* field, void** arg) {
    std::vector<float>* vec = static_cast<std::vector<float>*>(*arg);

    float value;
    if (!pb_decode_float(stream, &value)) {
      return false;
    }

    vec->push_back(value);
    return true;
  }

  // JointTrajectoryPoint数组编码回调
  static bool encode_joint_trajectory_point_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg) {
    const std::vector<JointTrajectoryPoint>* points = static_cast<const std::vector<JointTrajectoryPoint>*>(*arg);

    for (const JointTrajectoryPoint& point : *points) {
      if (!pb_encode_tag_for_field(stream, field)) {
        return false;
      }

      // 编码单个JointTrajectoryPoint
      ::JointTrajectoryPoint proto_point;

      // 设置回调函数
      proto_point.positions.funcs.encode = encode_float_array_callback;
      proto_point.positions.arg = const_cast<std::vector<float>*>(&point.positions);

      proto_point.velocities.funcs.encode = encode_float_array_callback;
      proto_point.velocities.arg = const_cast<std::vector<float>*>(&point.velocities);

      proto_point.accelerations.funcs.encode = encode_float_array_callback;
      proto_point.accelerations.arg = const_cast<std::vector<float>*>(&point.accelerations);

      proto_point.effort.funcs.encode = encode_float_array_callback;
      proto_point.effort.arg = const_cast<std::vector<float>*>(&point.effort);

      if (point.time_from_start.seconds != 0 || point.time_from_start.nanoseconds != 0) {
        proto_point.has_time_from_start = true;
        proto_point.time_from_start.seconds = point.time_from_start.seconds;
        proto_point.time_from_start.nanoseconds = point.time_from_start.nanoseconds;
      }

      if (!pb_encode_submessage(stream, ::JointTrajectoryPoint_fields, &proto_point)) {
        return false;
      }
    }
    return true;
  }

  // JointTrajectoryPoint数组解码回调
  static bool decode_joint_trajectory_point_callback(pb_istream_t* stream, const pb_field_t* field, void** arg) {
    std::vector<JointTrajectoryPoint>* points = static_cast<std::vector<JointTrajectoryPoint>*>(*arg);

    JointTrajectoryPoint point;
    ::JointTrajectoryPoint proto_point = ::JointTrajectoryPoint_init_zero;

    // 设置回调函数
    proto_point.positions.funcs.decode = decode_float_array_callback;
    proto_point.positions.arg = &point.positions;

    proto_point.velocities.funcs.decode = decode_float_array_callback;
    proto_point.velocities.arg = &point.velocities;

    proto_point.accelerations.funcs.decode = decode_float_array_callback;
    proto_point.accelerations.arg = &point.accelerations;

    proto_point.effort.funcs.decode = decode_float_array_callback;
    proto_point.effort.arg = &point.effort;

    if (!pb_decode(stream, ::JointTrajectoryPoint_fields, &proto_point)) {
      return false;
    }

    if (proto_point.has_time_from_start) {
      point.time_from_start.seconds = proto_point.time_from_start.seconds;
      point.time_from_start.nanoseconds = proto_point.time_from_start.nanoseconds;
    }

    points->push_back(point);
    return true;
  }

 public:
  /**
   * 序列化 LogMessage
   */
  static std::string serialize(const LogMessage& message) {
    ::LogMessage proto_msg = ::LogMessage_init_zero;

    proto_msg.log_level = static_cast<::LogLevel>(message.log_level);
    proto_msg.str.funcs.encode = encode_string_callback;
    proto_msg.str.arg = const_cast<std::string*>(&message.str);

    NanoPb::StringOutputStream stream;
    if (!pb_encode(&stream, ::LogMessage_fields, &proto_msg)) {
      return "";
    }

    auto buffer = stream.release();
    return buffer ? *buffer : "";
  }

  /**
   * 反序列化 LogMessage
   */
  static bool deserialize(const std::string& data, LogMessage& message) {
    ::LogMessage proto_msg = ::LogMessage_init_zero;

    proto_msg.str.funcs.decode = decode_string_callback;
    proto_msg.str.arg = &message.str;

    auto buffer = std::make_unique<std::string>(data);
    NanoPb::StringInputStream stream(std::move(buffer));

    if (!pb_decode(&stream, ::LogMessage_fields, &proto_msg)) {
      return false;
    }

    message.log_level = static_cast<LogLevel>(proto_msg.log_level);
    return true;
  }

  /**
   * 序列化 JointTrajectory
   */
  static std::string serialize(const JointTrajectory& trajectory) {
    ::JointTrajectory proto_traj = ::JointTrajectory_init_zero;

    proto_traj.points.funcs.encode = encode_joint_trajectory_point_callback;
    proto_traj.points.arg = const_cast<std::vector<JointTrajectoryPoint>*>(&trajectory.points);

    NanoPb::StringOutputStream stream;
    if (!pb_encode(&stream, ::JointTrajectory_fields, &proto_traj)) {
      return "";
    }

    auto buffer = stream.release();
    return buffer ? *buffer : "";
  }

  /**
   * 反序列化 JointTrajectory
   */
  static bool deserialize(const std::string& data, JointTrajectory& trajectory) {
    ::JointTrajectory proto_traj = ::JointTrajectory_init_zero;

    proto_traj.points.funcs.decode = decode_joint_trajectory_point_callback;
    proto_traj.points.arg = &trajectory.points;

    auto buffer = std::make_unique<std::string>(data);
    NanoPb::StringInputStream stream(std::move(buffer));

    return pb_decode(&stream, ::JointTrajectory_fields, &proto_traj);
  }

  /**
   * 序列化 SbusFrame
   */
  static std::string serialize(const SbusFrame& frame) {
    ::SbusFrame proto_frame = ::SbusFrame_init_zero;

    proto_frame.ch.funcs.encode = encode_float_array_callback;
    proto_frame.ch.arg = const_cast<std::vector<float>*>(&frame.ch);

    NanoPb::StringOutputStream stream;
    if (!pb_encode(&stream, ::SbusFrame_fields, &proto_frame)) {
      return "";
    }

    auto buffer = stream.release();
    return buffer ? *buffer : "";
  }

  /**
   * 反序列化 SbusFrame
   */
  static bool deserialize(const std::string& data, SbusFrame& frame) {
    ::SbusFrame proto_frame = ::SbusFrame_init_zero;

    proto_frame.ch.funcs.decode = decode_float_array_callback;
    proto_frame.ch.arg = &frame.ch;

    auto buffer = std::make_unique<std::string>(data);
    NanoPb::StringInputStream stream(std::move(buffer));

    return pb_decode(&stream, ::SbusFrame_fields, &proto_frame);
  }

  /**
   * 序列化 Duration
   */
  static std::string serialize(const Duration& duration) {
    ::Duration proto_duration;
    proto_duration.seconds = duration.seconds;
    proto_duration.nanoseconds = duration.nanoseconds;

    NanoPb::StringOutputStream stream;
    if (!pb_encode(&stream, ::Duration_fields, &proto_duration)) {
      return "";
    }

    auto buffer = stream.release();
    return buffer ? *buffer : "";
  }

  /**
   * 反序列化 Duration
   */
  static bool deserialize(const std::string& data, Duration& duration) {
    ::Duration proto_duration;

    auto buffer = std::make_unique<std::string>(data);
    NanoPb::StringInputStream stream(std::move(buffer));

    if (!pb_decode(&stream, ::Duration_fields, &proto_duration)) {
      return false;
    }

    duration.seconds = proto_duration.seconds;
    duration.nanoseconds = proto_duration.nanoseconds;
    return true;
  }
};

}  // namespace proto
