#pragma once

#include <functional>

#include <librm.hpp>

/**
 * 底盘发，机械臂收
 */
struct __attribute__((__packed__)) CustomControllerCommand {
  bool is_cartesian_command : 1;
  bool ok : 1;
  struct __attribute__((__packed__)) {
    bool a : 1;
    bool b : 1;
    bool c : 1;
    bool d : 1;
    bool reserved_1 : 1;
    bool reserved_2 : 1;
  } buttons{};
  float joint_positions[6];
};
struct __attribute__((__packed__)) CartesianCustomControllerCommand {
  bool is_cartesian_command : 1;
  bool ok : 1;
  bool btn_a : 1;
  bool btn_b : 1;
  bool reserved_1 : 1;
  bool reserved_2 : 1;
  bool reserved_3 : 1;
  bool reserved_4 : 1;
  float translation_xyz[3];
  float rotation_wxyz[4];
};

/**
 * 机械臂发，底盘收，然后转发到图传链路串口上，自定义控制器收
 */
struct __attribute__((__packed__)) JointPosition {
  float joint_positions[6];
};

/**
 * 底盘发，机械臂收
 */
using VtRemoteControl = rm::device::RefereeProtocol<rm::device::RefereeRevision::kV170>::RemoteControl;

// clang-format off
static_assert(sizeof(CustomControllerCommand) <= 64, "CustomControllerCommand struct doesn't fit in a single CANFD frame!");
static_assert(sizeof(JointPosition) <= 64, "JointPosition struct doesn't fit in a single CANFD frame!");
static_assert(sizeof(VtRemoteControl) <= 64, "VtRemoteControl struct doesn't fit in a single CANFD frame!");
// clang-format on

class VtLinkForwarder : public rm::device::CanDevice {
 public:
  constexpr static uint16_t kBaseFrameId = 0x5f;
  enum FrameIdOffset : uint16_t {
    kCustomCtrl = 0u,
    kJointPosition,
    kVtRemote,
  };

  explicit VtLinkForwarder(rm::hal::CanInterface &can) : CanDevice(can, kBaseFrameId + FrameIdOffset::kJointPosition) {}

  void WriteCustomCtrl(const uint8_t *data, size_t len) const {
    can_->Write(kBaseFrameId + FrameIdOffset::kCustomCtrl, data, len);
  }

  void Write(const VtRemoteControl &data) const {
    can_->Write(kBaseFrameId + FrameIdOffset::kVtRemote, reinterpret_cast<const uint8_t *>(&data), sizeof(data));
  }

  void RxCallback(const rm::hal::CanFrame *msg) override {
    switch (msg->rx_std_id) {
      case kBaseFrameId + FrameIdOffset::kJointPosition: {
        ReportStatus(kOk);
        std::memcpy(&joint_position_data_, msg->data.data(), sizeof(joint_position_data_));
        if (on_joint_position_recv_cb_) {
          on_joint_position_recv_cb_();
        }
        break;
      }
      default: {
        break;
      }
    }
  }

 private:
  JointPosition joint_position_data_{};
  std::function<void()> on_joint_position_recv_cb_;
};

class VtLinkReceiver : public rm::device::CanDevice {
 public:
  explicit VtLinkReceiver(rm::hal::CanInterface &can)
      : CanDevice(can, VtLinkForwarder::kBaseFrameId + VtLinkForwarder::FrameIdOffset::kCustomCtrl,
                  VtLinkForwarder::kBaseFrameId + VtLinkForwarder::FrameIdOffset::kVtRemote) {}

  void OnCustomCtrlRecv(std::function<void()> cb) { on_custom_ctrl_recv_cb_ = std::move(cb); }
  void OnCartesianCustomCtrlRecv(std::function<void()> cb) { on_cartesian_custom_ctrl_recv_cb_ = std::move(cb); }
  void OnVtRemoteRecv(std::function<void()> cb) { on_vt_remote_recv_cb_ = std::move(cb); }

  const auto &custom_ctrl_data() const { return custom_ctrl_data_; }
  const auto &cartesian_custom_ctrl_data() const { return cartesian_custom_ctrl_data_; }
  const auto &vt_remote_data() const { return vt_remote_data_; }

 protected:
  void RxCallback(const rm::hal::CanFrame *msg) override {
    switch (msg->rx_std_id) {
      case VtLinkForwarder::kBaseFrameId + VtLinkForwarder::FrameIdOffset::kCustomCtrl: {
        ReportStatus(kOk);
        // 判断is_cartesian_command字段
        const bool is_cartesian = (msg->data[0] & 0x01) != 0;
        if (is_cartesian) {
          std::memcpy(&cartesian_custom_ctrl_data_, msg->data.data(), sizeof(cartesian_custom_ctrl_data_));
          if (on_cartesian_custom_ctrl_recv_cb_) {
            on_cartesian_custom_ctrl_recv_cb_();
          }
        } else {
          std::memcpy(&custom_ctrl_data_, msg->data.data(), sizeof(custom_ctrl_data_));
          if (on_custom_ctrl_recv_cb_) {
            on_custom_ctrl_recv_cb_();
          }
        }
        break;
      }
      case VtLinkForwarder::kBaseFrameId + VtLinkForwarder::FrameIdOffset::kVtRemote: {
        ReportStatus(kOk);
        std::memcpy(&vt_remote_data_, msg->data.data(), sizeof(vt_remote_data_));
        if (on_vt_remote_recv_cb_) {
          on_vt_remote_recv_cb_();
        }
        break;
      }
      default: {
        break;
      }
    }
  }

  CustomControllerCommand custom_ctrl_data_{};
  CartesianCustomControllerCommand cartesian_custom_ctrl_data_{};
  VtRemoteControl vt_remote_data_{};
  std::function<void()> on_custom_ctrl_recv_cb_;
  std::function<void()> on_cartesian_custom_ctrl_recv_cb_;
  std::function<void()> on_vt_remote_recv_cb_;
};