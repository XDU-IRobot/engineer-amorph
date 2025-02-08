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

#include <memory>
#include <random>
#include <cxxabi.h>

#include <tinyfsm.hpp>
#include <librm.hpp>

#include "fluorite_msgs/msg/chassis_state.hpp"
#include "common/logger.hpp"

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
        rr_steer_position{nullptr};
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
};

class NoForce : public fsm::Chassis {
 public:
  void react(event::chassis::Control const &control) override;
  void entry() override;
};
}  // namespace chassis
}  // namespace state

#endif