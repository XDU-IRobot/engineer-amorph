/**
 * @file  fsm.hpp
 * @brief 状态机定义，使用有限状态机库tinyfsm
 */

#ifndef ARM_FSM_HPP
#define ARM_FSM_HPP

#include <memory>
#include <random>

#include <tinyfsm.hpp>
#include <librm.hpp>

#include "fluorite_msgs/msg/arm_state.hpp"
#include "common/logger.hpp"

namespace event {
namespace arm {
template <typename T>
struct SwitchMode : tinyfsm::Event {};
struct Control {};
}  // namespace arm
}  // namespace event

namespace fsm {

class Arm : public tinyfsm::MooreMachine<Arm> {
 public:
  void react(tinyfsm::Event const &event);

  virtual void react(event::arm::Control const &control);
  virtual void entry() = 0;

  template <typename Mode>
  void react(event::arm::SwitchMode<Mode> const &) {
    transit<Mode>();
    info(("MODE SWITCH: " + std::string(Mode::name)).c_str());
  }

 public:
  /**
   * @brief   状态向量，数据更新要及时填进这里
   * @note    角度单位rad，转速单位rpm
   * @note    先按定义、范围把数据处理好再填进来
   */
  static fluorite_msgs::msg::ArmState state;

  /**
   * @brief   执行器（电机）
   */
  static struct Actuator {
    std::unique_ptr<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>> end_yaw_1{nullptr}, end_pitch{nullptr},
        end_yaw_2{nullptr};
  } actuator;

  /**
   * @brief   控制器（算法对象）
   */
  static struct Controller {
    ;
  } controller;
};

using fsm_list = tinyfsm::FsmList<Arm>;

template <typename Event>
void DispatchEvent(Event const &event) {
  fsm_list::dispatch<Event>(event);
}

}  // namespace fsm

namespace state {
namespace arm {

class Debug : public fsm::Arm {
 public:
  constexpr static char const *name = "Debug";
  void entry() override;
};

class NoForce : public fsm::Arm {
 public:
  constexpr static char const *name = "NoForce";
  void react(event::arm::Control const &control) override;
  void entry() override;
};
}  // namespace arm
}  // namespace state

#endif