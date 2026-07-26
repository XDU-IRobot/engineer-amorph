#pragma once

#include <etl/unordered_map.h>
#include <librm.hpp>

#include "broker.hpp"

namespace xbox_btn {

constexpr int BTN_A = 0;
constexpr int BTN_B = 1;
constexpr int BTN_X = 2;
constexpr int BTN_Y = 3;
constexpr int BTN_LB = 4;
constexpr int BTN_RB = 5;
constexpr int BTN_VIEW = 6;
constexpr int BTN_MENU = 7;
constexpr int BTN_SHARE = 8;
constexpr int BTN_XBOX = 9;
constexpr int BTN_LS = 10;
constexpr int BTN_RS = 11;
constexpr int DPAD_UP = 12;
constexpr int DPAD_DOWN = 13;
constexpr int DPAD_LEFT = 14;
constexpr int DPAD_RIGHT = 15;

}  // namespace xbox_btn

/**
 * @brief Xbox 手柄控制器，基于 CustomClientBroker 接收数据并封装按键/摇杆访问
 * @note 按键回调注册方式与 WflyET16s 一致：xbox(xbox_btn::BTN_A).OnValueChange([](const bool &old, const bool &new_val)
 * {...})
 */
class XboxController : public rm::device::Device {
 public:
  explicit XboxController(CustomClientBroker &broker) {
    broker.OnJoyRecv([this](const amorph::schema::Joy &data) {
      ReportStatus(kOk);
      joy_ = data;
    });
  }

  void Update() {
    for (auto &[btn, watcher] : button_watchers_) {
      watcher.Update(GetButton(btn));
    }
  }

  /// @name 摇杆轴值（归一化到 [-1.0, +1.0]）
  /// @{
  float left_x() const { return joy_.left_x; }
  float left_y() const { return joy_.left_y; }
  float right_x() const { return joy_.right_x; }
  float right_y() const { return joy_.right_y; }
  /// @}

  /// @name 扳机值（归一化到 [0.0, 1.0]）
  /// @{
  float lt() const { return joy_.lt; }
  float rt() const { return joy_.rt; }
  /// @}

  /// @brief 获取指定按键的当前 bool 状态
  bool GetButton(int btn) const {
    switch (btn) {
      case xbox_btn::BTN_A:
        return joy_.btn_a;
      case xbox_btn::BTN_B:
        return joy_.btn_b;
      case xbox_btn::BTN_X:
        return joy_.btn_x;
      case xbox_btn::BTN_Y:
        return joy_.btn_y;
      case xbox_btn::BTN_LB:
        return joy_.btn_lb;
      case xbox_btn::BTN_RB:
        return joy_.btn_rb;
      case xbox_btn::BTN_VIEW:
        return joy_.btn_view;
      case xbox_btn::BTN_MENU:
        return joy_.btn_menu;
      case xbox_btn::BTN_SHARE:
        return joy_.btn_share;
      case xbox_btn::BTN_XBOX:
        return joy_.btn_xbox;
      case xbox_btn::BTN_LS:
        return joy_.btn_ls;
      case xbox_btn::BTN_RS:
        return joy_.btn_rs;
      case xbox_btn::DPAD_UP:
        return joy_.dpad_up;
      case xbox_btn::DPAD_DOWN:
        return joy_.dpad_down;
      case xbox_btn::DPAD_LEFT:
        return joy_.dpad_left;
      case xbox_btn::DPAD_RIGHT:
        return joy_.dpad_right;
      default:
        return false;
    }
  }

  /// @brief 获取按键边沿检测器，用于注册 OnValueChange 回调（与 WflyET16s 的 operator()(int ch) 用法一致）
  auto &operator()(int btn) { return button_watchers_[btn]; }

  /// @brief 获取原始 Joy 数据
  const auto &data() const { return joy_; }

 private:
  amorph::schema::Joy joy_;

  etl::unordered_map<int, rm::modules::SparseValueWatcher<bool, true>, 16> button_watchers_{
      {xbox_btn::BTN_A, {}},     {xbox_btn::BTN_B, {}},     {xbox_btn::BTN_X, {}},     {xbox_btn::BTN_Y, {}},
      {xbox_btn::BTN_LB, {}},    {xbox_btn::BTN_RB, {}},    {xbox_btn::BTN_VIEW, {}},  {xbox_btn::BTN_MENU, {}},
      {xbox_btn::BTN_SHARE, {}}, {xbox_btn::BTN_XBOX, {}},  {xbox_btn::BTN_LS, {}},    {xbox_btn::BTN_RS, {}},
      {xbox_btn::DPAD_UP, {}},   {xbox_btn::DPAD_DOWN, {}}, {xbox_btn::DPAD_LEFT, {}}, {xbox_btn::DPAD_RIGHT, {}},
  };
};