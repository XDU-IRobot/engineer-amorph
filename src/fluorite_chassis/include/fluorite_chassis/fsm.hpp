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

#ifndef FSM_HPP
#define FSM_HPP

#include <memory>
#include <random>

#include <tinyfsm.hpp>
#include <librm.hpp>

#include "fluorite_msgs/msg/chassis_state.hpp"
#include "fluorite_chassis/logger.hpp"
#include "fluorite_chassis/chassis_fk.hpp"
#include "fluorite_chassis/encoder_counter.hpp"

namespace event {
namespace chassis {
template <typename T>
struct SwitchMode : tinyfsm::Event {};
struct Control {
  float vx, vy, vw;
};
}  // namespace chassis
}  // namespace event

namespace fsm {

class Chassis : public tinyfsm::MooreMachine<Chassis> {
 public:
  void react(tinyfsm::Event const &event);

  virtual void react(event::chassis::Control const &control);
  virtual void entry() = 0;

  template <typename Mode>
  void react(event::chassis::SwitchMode<Mode> const &) {
    transit<Mode>();
    info(("MODE SWITCH: " + std::string(Mode::name)).c_str());
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
    std::unique_ptr<rm::device::M3508> l_front_wheel{nullptr}, l_mid_wheel{nullptr}, l_rear_wheel{nullptr},
        l_lift{nullptr}, r_front_wheel{nullptr}, r_mid_wheel{nullptr}, r_rear_wheel{nullptr}, r_lift{nullptr};
  } actuator;

  /**
   * @brief   控制器（算法对象）
   */
  static struct Controller {
    std::unique_ptr<rm::modules::algorithm::PID> l_front_speed{nullptr}, l_mid_speed{nullptr}, l_rear_speed{nullptr},
        l_lift_speed{nullptr}, r_front_speed{nullptr}, r_mid_speed{nullptr}, r_rear_speed{nullptr},
        r_lift_speed{nullptr}, l_lift_position{nullptr}, r_lift_position{nullptr};
    ChassisFk chassis_solver{0.54, 0.44};
    EncoderCounter<int> l_lift_encoder{8191}, r_lift_encoder{8191};
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

class Neutral : public fsm::Chassis {
 public:
  constexpr static char const *name = "Neutral";
  void entry() override;
};

class Lift : public fsm::Chassis {
 public:
  constexpr static char const *name = "Lift";
  void entry() override;
};

class Lower : public fsm::Chassis {
 public:
  constexpr static char const *name = "Lower";
  void entry() override;
};

class Adaptive : public fsm::Chassis {
 public:
  constexpr static char const *name = "Adaptive";
  void entry() override;
};

class NoForce : public fsm::Chassis {
 public:
  constexpr static char const *name = "NoForce";
  void react(event::chassis::Control const &control) override;
  void entry() override;
};
}  // namespace chassis
}  // namespace state

#endif