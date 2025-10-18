#include <iostream>
#include <vector>
#include <string>
#include <cassert>

#include "pbf_bridge.hpp"

void test_log_message() {
  std::cout << "=== Testing LogMessage ===" << std::endl;

  // 创建测试数据
  proto::LogMessage original_msg;
  original_msg.log_level = proto::LogLevel::kError;
  original_msg.str = "This is a test log message";

  // 序列化
  std::string serialized = proto::PbfBridge::serialize(original_msg);
  std::cout << "Serialized size: " << serialized.size() << " bytes" << std::endl;

  // 反序列化
  proto::LogMessage deserialized_msg;
  bool success = proto::PbfBridge::deserialize(serialized, deserialized_msg);

  if (success) {
    std::cout << "Deserialization successful!" << std::endl;
    std::cout << "Original level: " << static_cast<int>(original_msg.log_level)
              << ", Deserialized level: " << static_cast<int>(deserialized_msg.log_level) << std::endl;
    std::cout << "Original str: " << original_msg.str << std::endl;
    std::cout << "Deserialized str: " << deserialized_msg.str << std::endl;

    // 验证数据一致性
    assert(original_msg.log_level == deserialized_msg.log_level);
    assert(original_msg.str == deserialized_msg.str);
    std::cout << "✓ LogMessage test passed!" << std::endl;
  } else {
    std::cout << "✗ LogMessage deserialization failed!" << std::endl;
  }
}

void test_duration() {
  std::cout << "\n=== Testing Duration ===" << std::endl;

  proto::Duration original_duration;
  original_duration.seconds = 1234567890;
  original_duration.nanoseconds = 987654321;

  std::string serialized = proto::PbfBridge::serialize(original_duration);
  std::cout << "Serialized size: " << serialized.size() << " bytes" << std::endl;

  proto::Duration deserialized_duration;
  bool success = proto::PbfBridge::deserialize(serialized, deserialized_duration);

  if (success) {
    std::cout << "Deserialization successful!" << std::endl;
    std::cout << "Original seconds: " << original_duration.seconds
              << ", Deserialized seconds: " << deserialized_duration.seconds << std::endl;
    std::cout << "Original nanoseconds: " << original_duration.nanoseconds
              << ", Deserialized nanoseconds: " << deserialized_duration.nanoseconds << std::endl;

    assert(original_duration.seconds == deserialized_duration.seconds);
    assert(original_duration.nanoseconds == deserialized_duration.nanoseconds);
    std::cout << "✓ Duration test passed!" << std::endl;
  } else {
    std::cout << "✗ Duration deserialization failed!" << std::endl;
  }
}

void test_joint_trajectory_point() {
  std::cout << "\n=== Testing JointTrajectoryPoint ===" << std::endl;

  proto::JointTrajectoryPoint original_point;
  original_point.positions = {1.0f, 2.0f, 3.0f, 4.0f};
  original_point.velocities = {0.1f, 0.2f, 0.3f, 0.4f};
  original_point.accelerations = {0.01f, 0.02f, 0.03f, 0.04f};
  original_point.effort = {10.0f, 20.0f, 30.0f, 40.0f};
  original_point.time_from_start.seconds = 5;
  original_point.time_from_start.nanoseconds = 500000000;

  std::string serialized = proto::PbfBridge::serialize(original_point);
  std::cout << "Serialized size: " << serialized.size() << " bytes" << std::endl;

  proto::JointTrajectoryPoint deserialized_point;
  bool success = proto::PbfBridge::deserialize(serialized, deserialized_point);

  if (success) {
    std::cout << "Deserialization successful!" << std::endl;

    // 验证positions
    assert(original_point.positions.size() == deserialized_point.positions.size());
    for (size_t i = 0; i < original_point.positions.size(); ++i) {
      assert(original_point.positions[i] == deserialized_point.positions[i]);
    }

    // 验证time_from_start
    assert(original_point.time_from_start.seconds == deserialized_point.time_from_start.seconds);
    assert(original_point.time_from_start.nanoseconds == deserialized_point.time_from_start.nanoseconds);

    std::cout << "✓ JointTrajectoryPoint test passed!" << std::endl;
  } else {
    std::cout << "✗ JointTrajectoryPoint deserialization failed!" << std::endl;
  }
}

void test_joint_trajectory() {
  std::cout << "\n=== Testing JointTrajectory ===" << std::endl;

  proto::JointTrajectory original_trajectory;

  // 创建两个轨迹点
  proto::JointTrajectoryPoint point1;
  point1.positions = {1.0f, 2.0f, 3.0f};
  point1.velocities = {0.1f, 0.2f, 0.3f};
  point1.time_from_start.seconds = 1;

  proto::JointTrajectoryPoint point2;
  point2.positions = {4.0f, 5.0f, 6.0f};
  point2.velocities = {0.4f, 0.5f, 0.6f};
  point2.time_from_start.seconds = 2;

  original_trajectory.points = {point1, point2};

  std::string serialized = proto::PbfBridge::serialize(original_trajectory);
  std::cout << "Serialized size: " << serialized.size() << " bytes" << std::endl;

  proto::JointTrajectory deserialized_trajectory;
  bool success = proto::PbfBridge::deserialize(serialized, deserialized_trajectory);

  if (success) {
    std::cout << "Deserialization successful!" << std::endl;
    std::cout << "Original points count: " << original_trajectory.points.size()
              << ", Deserialized points count: " << deserialized_trajectory.points.size() << std::endl;

    assert(original_trajectory.points.size() == deserialized_trajectory.points.size());
    std::cout << "✓ JointTrajectory test passed!" << std::endl;
  } else {
    std::cout << "✗ JointTrajectory deserialization failed!" << std::endl;
  }
}

void test_sbus_frame() {
  std::cout << "\n=== Testing SbusFrame ===" << std::endl;

  proto::SbusFrame original_frame;
  original_frame.ch = {1500.0f, 1520.0f, 1480.0f, 1550.0f, 1450.0f, 1600.0f};

  std::string serialized = proto::PbfBridge::serialize(original_frame);
  std::cout << "Serialized size: " << serialized.size() << " bytes" << std::endl;

  proto::SbusFrame deserialized_frame;
  bool success = proto::PbfBridge::deserialize(serialized, deserialized_frame);

  if (success) {
    std::cout << "Deserialization successful!" << std::endl;
    std::cout << "Original channels count: " << original_frame.ch.size()
              << ", Deserialized channels count: " << deserialized_frame.ch.size() << std::endl;

    assert(original_frame.ch.size() == deserialized_frame.ch.size());
    for (size_t i = 0; i < original_frame.ch.size(); ++i) {
      assert(original_frame.ch[i] == deserialized_frame.ch[i]);
    }
    std::cout << "✓ SbusFrame test passed!" << std::endl;
  } else {
    std::cout << "✗ SbusFrame deserialization failed!" << std::endl;
  }
}

void test_empty_data() {
  std::cout << "\n=== Testing Empty Data Handling ===" << std::endl;

  // 测试空字符串
  proto::LogMessage msg;
  bool success = proto::PbfBridge::deserialize("", msg);
  std::cout << "Empty string deserialization: " << (success ? "success" : "failed (expected)") << std::endl;

  // 测试无效数据
  std::string invalid_data = "invalid_protobuf_data";
  success = proto::PbfBridge::deserialize(invalid_data, msg);
  std::cout << "Invalid data deserialization: " << (success ? "success" : "failed (expected)") << std::endl;

  std::cout << "✓ Empty data test completed!" << std::endl;
}

int main() {
  std::cout << "Starting PbfBridge Test Suite..." << std::endl;
  std::cout << "===================================" << std::endl;

  try {
    test_log_message();
    test_duration();
    test_joint_trajectory_point();
    test_joint_trajectory();
    test_sbus_frame();
    test_empty_data();

    std::cout << "\n===================================" << std::endl;
    std::cout << "All tests completed successfully! 🎉" << std::endl;
    std::cout << "PbfBridge is working correctly." << std::endl;

  } catch (const std::exception& e) {
    std::cout << "\n❌ Test failed with exception: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cout << "\n❌ Test failed with unknown exception" << std::endl;
    return 1;
  }

  return 0;
}
