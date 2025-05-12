/**
 * @file  fsm.hpp
 * @brief 本文件的内容是底盘状态机的定义，使用了一个有限状态机库tinyfsm（先去看这个库怎么用）
 *
 * @brief 底盘状态机的状态有Spin, FollowGimbal, NoForce三种
 *
 * @brief 状态机要对两种事件做出反应：控制事件Control和模式切换事件SwitchMode
 * @brief 接收到Control事件后，状态机会执行一次控制循环，所以必须持续在主线程里dispatch Control事件
 * @brief 接收到SwitchMode事件后，对应的状态机会切换到指定的状态
 */

#ifndef CHASSIS_FSM_HPP
#define CHASSIS_FSM_HPP

#include <atomic>
#include <memory>
#include <cxxabi.h>

#include <tinyfsm.hpp>
#include <librm.hpp>

#include "keyboard_msgs/msg/key.hpp"

#include "fluorite_msgs/msg/chassis_state.hpp"
#include "common/keypress_handler.hpp"
#include "common/logger.hpp"

namespace event {
namespace chassis {
template <typename T>
struct SwitchMode : tinyfsm::Event {};
/**
 * @brief 所有链路的控制量全都在这里，数据由节点的各个subscriber回调函数负责更新
 */
struct Control : tinyfsm::Event {
  Control() {
    // map里的std::atomic<bool>不能用std::initializer_list初始化，必须一个一个构造
    for (const auto &keyboard_map_k : {
             keyboard_msgs::msg::Key::KEY_W,
             keyboard_msgs::msg::Key::KEY_S,
             keyboard_msgs::msg::Key::KEY_A,
             keyboard_msgs::msg::Key::KEY_D,
             keyboard_msgs::msg::Key::KEY_Q,
             keyboard_msgs::msg::Key::KEY_E,
             keyboard_msgs::msg::Key::KEY_R,
             keyboard_msgs::msg::Key::KEY_F,
             keyboard_msgs::msg::Key::KEY_G,
             keyboard_msgs::msg::Key::KEY_Z,
             keyboard_msgs::msg::Key::KEY_X,
             keyboard_msgs::msg::Key::KEY_C,
             keyboard_msgs::msg::Key::KEY_V,
             keyboard_msgs::msg::Key::KEY_B,
         }) {
      keyboard_key.try_emplace(keyboard_map_k, false);
    }
  }
  std::atomic<double> mouse_x;                      ///< 鼠标左右，范围[-1, 1] * horizontal_sensitivity
  std::atomic<double> mouse_y;                      ///< 鼠标上下，范围[-1, 1] * vertical_sensitivity
  std::atomic<double> mouse_wheel;                  ///< 鼠标滚轮，范围[-1, 1]
  std::atomic<double> left_x;                       ///< 左摇杆左右，范围[-1, 1] * horizontal_sensitivity
  std::atomic<double> left_y;                       ///< 左摇杆上下，范围[-1, 1] * vertical_sensitivity
  std::atomic<double> right_x;                      ///< 右摇杆左右，范围[-1, 1] * horizontal_sensitivity
  std::atomic<double> right_y;                      ///< 右摇杆上下，范围[-1, 1] * vertical_sensitivity
  std::atomic<double> ld, rd;                       ///< 左右旋钮，范围[-1, 1]，逆负顺正s
  std::atomic<double> ls, rs;                       ///< 遥控器背面的左右滑块，范围[-1, 1]，下负上正
  std::atomic<int> sa, sb, sc, sd, se, sf, sg, sh;  ///< 拨杆，上1中0下-1
  std::unordered_map<uint16_t, std::atomic<bool>> keyboard_key;  ///< 键盘按键
  std::atomic<uint16_t> keyboard_modifier;                       ///< 键盘组合键modifier
  // struct _ {
  //   _(Control *this_) : this_(this_) {}
  //   Hotkey switch_arm_pump{[&]() -> bool {
  //     return this_->keyboard_key.at(keyboard_msgs::msg::Key::KEY_L) &&
  //            this_->keyboard_key.at(keyboard_msgs::msg::Key::KEY_LSHIFT);
  //   }};

  //  private:
  //   Control *this_;
  // } hotkeys;
};
}  // namespace chassis
}  // namespace event

namespace fsm {

class Chassis : public tinyfsm::MooreMachine<Chassis> {
 public:
  void react(tinyfsm::Event const &event);

  virtual void entry() = 0;
  virtual void react(event::chassis::Control const &control) = 0;

  template <typename Mode>
  void react(event::chassis::SwitchMode<Mode> const &) {
    if (this->is_in_state<Mode>()) {
      return;
    }
    transit<Mode>();
    info("MODE SWITCH: %s", abi::__cxa_demangle(typeid(Mode).name(), nullptr, nullptr, nullptr));
  }

 public:
  /**
   * @brief   状态向量，数据更新要及时填进这里
   * @note    角度单位rad，转速单位rpm
   * @note    先按定义、范围把数据处理好再填进来
   */
  static fluorite_msgs::msg::ChassisState state;

  /**
   * @brief   执行器（电机）
   */
  static struct Actuator {
    std::unique_ptr<rm::device::DirectDriveMotor> base_yaw{nullptr};
    std::unique_ptr<rm::device::M3508> lf_wheel{nullptr}, rf_wheel{nullptr}, lr_wheel{nullptr}, rr_wheel{nullptr};
    std::unique_ptr<rm::device::GM6020> lf_steer{nullptr}, rf_steer{nullptr}, lr_steer{nullptr}, rr_steer{nullptr};
  } actuator;

  /**
   * @brief   控制器（算法对象）
   */
  static struct Controller {
   private:
    using PosPid = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;
    using PosRingPid = rm::modules::algorithm::RingPID<rm::modules::algorithm::PIDType::kPosition>;

   public:
    std::unique_ptr<PosPid> lf_wheel_speed{nullptr}, rf_wheel_speed{nullptr}, lr_wheel_speed{nullptr},
        rr_wheel_speed{nullptr}, lf_steer_speed{nullptr}, rf_steer_speed{nullptr}, lr_steer_speed{nullptr},
        rr_steer_speed{nullptr};
    std::unique_ptr<PosRingPid> lf_steer_position{nullptr}, rf_steer_position{nullptr}, lr_steer_position{nullptr},
        rr_steer_position{nullptr}, chassis_orientation_position{nullptr};
    rm::modules::algorithm::SteeringChassis chassis_solver{0.5f};
  } controller;
};

using fsm_list = tinyfsm::FsmList<Chassis>;

template <typename Event>
void DispatchEvent(Event const &event) {
  fsm_list::dispatch<Event>(event);
}

}  // namespace fsm

namespace state {
namespace chassis {

class FollowGimbal : public fsm::Chassis {
 public:
  void entry() override;
  void react(event::chassis::Control const &control) override;
};

class NoForce : public fsm::Chassis {
 public:
  void entry() override;
  void react(event::chassis::Control const &control) override;
};
}  // namespace chassis
}  // namespace state

#endif