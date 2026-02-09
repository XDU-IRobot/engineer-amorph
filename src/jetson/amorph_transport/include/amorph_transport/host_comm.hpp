#pragma once

#include <librm.hpp>

#include <functional>

/**
 * @brief STM32通过CANFD总线向上位机发送的反馈
 */
struct __attribute__((__packed__)) Stm32Feedback {
  enum Mode : uint8_t {
    kNoForce,               ///< 强制无力
    kManual,                ///< 遥控器手动模式，调试/检录的时候用
    kIdle,                  ///< 空闲，可执行轨迹
    kHostControl,           ///< 正在被上位机控制
    kCustomCtrl,            ///< 自定义控制器操作
    kLongJointCalibrating,  ///< 超过位置编码器1圈的长关节校准位置(j6、夹爪）
    kFault,                 ///< 故障状态
  } mode;                     ///< 下位机目前所处的模式，定义见fsm.hpp
  float joint_positions[6];   ///< 当前关节位置，单位：rad
  float joint_velocities[6];  ///< 当前关节速度，单位：rad/s
  bool gripper_open;          ///< 夹爪开合状态，true表示开，false表示闭
};

/**
 * @brief 上位机通过CANFD总线向STM32发送的控制命令
 */
struct __attribute__((__packed__)) HostCommand {
  bool take_control;  ///< 是否接管控制权，true表示上位机接管控制权，false表示上位机放弃控制权
  float target_joint_positions[6];  ///< 上位机期望的目标关节位置，单位：rad
  bool gripper_open;                ///< 夹爪开合命令，true表示开，false表示闭
};

static_assert(sizeof(Stm32Feedback) <= 64, "Stm32Feedback struct doesn't fit in a single CANFD frame!");
static_assert(sizeof(HostCommand) <= 64, "HostCommand struct doesn't fit in a single CANFD frame!");

/**
 * @brief STM32视角的上位机类
 */
class Host : public rm::device::CanDevice {
 public:
  Host(rm::hal::CanInterface &can, size_t stm32_id) : CanDevice{can, 0x10}, stm32_id_{stm32_id} {}

  void Write() const { can_->Write(stm32_id_, reinterpret_cast<const uint8_t *>(&tx_data_), sizeof(tx_data_)); }

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
    std::memcpy(&rx_data_, msg->data.data(), sizeof(Stm32Feedback));
    if (on_rx_callback_) {
      on_rx_callback_(rx_data_);
    }
  }

  Stm32Feedback rx_data_{};
  size_t stm32_id_{};
  std::function<void(const Stm32Feedback &)> on_rx_callback_{nullptr};
};