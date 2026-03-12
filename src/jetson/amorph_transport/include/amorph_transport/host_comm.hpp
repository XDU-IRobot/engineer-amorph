#pragma once

#include <librm.hpp>

#include <atomic>
#include <functional>

/**
 * @brief STM32通过CANFD总线向上位机发送的反馈
 */
struct __attribute__((packed)) Stm32FeedbackCompressed {
  enum Mode : uint8_t {
    kNoForce,                            ///< 强制无力
    kManual,                             ///< 遥控器手动模式，调试/检录的时候用
    kIdle,                               ///< 空闲，可执行轨迹
    kHostControl,                        ///< 正在被上位机控制
    kCustomCtrl,                         ///< 自定义控制器操作
    kLongJointCalibrating,               ///< 超过位置编码器1圈的长关节校准位置(j6、夹爪）
    kFault,                              ///< 故障状态
  } mode;                                ///< 下位机目前所处的模式，定义见fsm.hpp
  rm::modules::f16 joint_positions[6];   ///< 当前关节位置，单位：rad
  rm::modules::f16 joint_velocities[6];  ///< 当前关节速度，单位：rad/s
  rm::modules::f16 joint_efforts[6];     ///< 当前关节力矩，单位：Nm，注意夹爪的effort没有实际意义，可以忽略
  bool gripper_open;                     ///< 夹爪开合状态，true表示开，false表示闭
};
struct Stm32Feedback {
  Stm32Feedback() = default;
  explicit Stm32Feedback(const Stm32FeedbackCompressed &compressed)
      : mode(compressed.mode), gripper_open(compressed.gripper_open) {
    for (size_t i = 0; i < 6; ++i) {
      joint_positions[i] = rm::modules::F16ToF32(compressed.joint_positions[i]);
      joint_velocities[i] = rm::modules::F16ToF32(compressed.joint_velocities[i]);
      joint_efforts[i] = rm::modules::F16ToF32(compressed.joint_efforts[i]);
    }
  }
  Stm32FeedbackCompressed ToCompressed() const {
    Stm32FeedbackCompressed compressed;
    compressed.mode = mode;
    compressed.gripper_open = gripper_open;
    for (size_t i = 0; i < 6; ++i) {
      compressed.joint_positions[i] = rm::modules::F32ToF16(joint_positions[i]);
      compressed.joint_velocities[i] = rm::modules::F32ToF16(joint_velocities[i]);
      compressed.joint_efforts[i] = rm::modules::F32ToF16(joint_efforts[i]);
    }
    return compressed;
  }
  Stm32FeedbackCompressed::Mode mode;  ///< 下位机目前所处的模式，定义见fsm.hpp
  float joint_positions[6];            ///< 当前关节位置，单位：rad
  float joint_velocities[6];           ///< 当前关节速度，单位：rad/s
  float joint_efforts[6];              ///< 当前关节力矩，单位：Nm，注意夹爪的effort没有实际意义，可以忽略
  bool gripper_open;                   ///< 夹爪开合状态，true表示开，false表示闭
};

/**
 * @brief 上位机通过CANFD总线向STM32发送的控制命令
 */
struct __attribute__((__packed__)) HostCommand {
  bool take_control;                ///< 是否接管控制权，true表示上位机接管控制权，false表示上位机放弃控制权
  float target_joint_positions[6];  ///< 上位机期望的目标关节位置，单位：rad
  bool gripper_open;                ///< 夹爪开合命令，true表示开，false表示闭
};

static_assert(sizeof(Stm32FeedbackCompressed) <= 64,
              "Stm32FeedbackCompressed struct doesn't fit in a single CANFD frame!");
static_assert(sizeof(HostCommand) <= 64, "HostCommand struct doesn't fit in a single CANFD frame!");

/**
 * @brief STM32视角的上位机类
 */
class Host : public rm::device::CanDevice {
 public:
  Host(rm::hal::CanInterface &can, size_t stm32_id) : CanDevice{can, 0x10}, stm32_id_{stm32_id} {}

  void Write() const {
    const Stm32FeedbackCompressed compressed = tx_data_.ToCompressed();
    can_->Write(stm32_id_, reinterpret_cast<const uint8_t *>(&compressed), sizeof(compressed));
  }

  void OnRx(std::function<void(const HostCommand &)> callback) { on_rx_callback_ = std::move(callback); }

  const auto &rx_data() const { return rx_data_; }

  Stm32Feedback tx_data_{};

 protected:
  void RxCallback(const rm::hal::CanFrame *msg) override {
    ReportStatus(kOk);
    std::memcpy(&rx_data_, msg->data.data(), sizeof(HostCommand));
    if (on_rx_callback_) {
      on_rx_callback_(rx_data_);
    }
  }

  HostCommand rx_data_{};
  size_t stm32_id_{};
  std::function<void(const HostCommand &)> on_rx_callback_{nullptr};
};

/**
 * @brief 上位机视角的STM32类
 */
class Stm32 : public rm::device::CanDevice {
 public:
  Stm32(rm::hal::CanInterface &can, size_t stm32_id) : CanDevice{can, stm32_id}, stm32_id_{stm32_id} {}

  void Write() const { can_->Write(0x10, reinterpret_cast<const uint8_t *>(&tx_data_), sizeof(tx_data_)); }

  void OnRx(std::function<void(const Stm32Feedback &)> callback) { on_rx_callback_ = std::move(callback); }

  const auto &rx_data() const { return rx_data_; }

  HostCommand tx_data_{};

 protected:
  void RxCallback(const rm::hal::CanFrame *msg) override {
    ReportStatus(kOk);
    const auto compressed = reinterpret_cast<const Stm32FeedbackCompressed *>(msg->data.data());
    rx_data_ = Stm32Feedback{*compressed};
    if (on_rx_callback_) {
      on_rx_callback_(rx_data_);
    }
  }

  Stm32Feedback rx_data_{};
  size_t stm32_id_{};
  std::function<void(const Stm32Feedback &)> on_rx_callback_{nullptr};
};