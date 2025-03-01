/**
 * @file  fsm.hpp
 * @brief 使用tinyfsm库实现的状态机定义，分为大yaw轴底座和机械臂两个状态机，每个state类对应一种模式，control
 * event驱动控制逻辑运行，switchmode event驱动状态转移
 */

#ifndef ARM_FSM_HPP
#define ARM_FSM_HPP

#include <atomic>
#include <memory>
#include <type_traits>

#include <tinyfsm.hpp>
#include <librm.hpp>

#include "fluorite_msgs/msg/arm_state.hpp"
#include "arm/pneumatic.hpp"
#include "arm/pid_trajectory_tracker.hpp"
#include "common/logger.hpp"

// 使用tinyfsm之后，就没办法再使用oop的各种编程范式来写代码了，因为状态机类没有对象
// 这里被迫创建了几个全局变量（定义都在fsm.cc里），用来存储状态机的外部依赖
// 如果以后找到了更好的实现方式，可以考虑重构
namespace fluorite {

/**
 * @brief   部分状态向量
 */
extern fluorite_msgs::msg::ArmState state;

/**
 * @brief   执行器（电机、气泵、电磁阀）
 */
struct Actuator {
  Actuator(rm::device::DirectDriveMotor &&base_yaw, rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> &&yaw,
           rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> &&pitch_1,
           rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> &&pitch_2,
           rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> &&end_yaw_1,
           rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> &&end_pitch, rm::device::GM6020 &&end_yaw_2,
           SuckerSystem &&sucker_system, StorageSystem &&storage_system)
      : base_yaw(std::move(base_yaw)),
        yaw(std::move(yaw)),
        pitch_1(std::move(pitch_1)),
        pitch_2(std::move(pitch_2)),
        end_yaw_1(std::move(end_yaw_1)),
        end_pitch(std::move(end_pitch)),
        end_yaw_2(std::move(end_yaw_2)),
        sucker_system(std::move(sucker_system)),
        storage_system(std::move(storage_system)) {}
  rm::device::DirectDriveMotor base_yaw;
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> yaw, pitch_1, pitch_2, end_yaw_1, end_pitch;
  rm::device::GM6020 end_yaw_2;
  SuckerSystem sucker_system;
  StorageSystem storage_system;
};
extern std::unique_ptr<Actuator> actuator;

/**
 * @brief   控制器（算法对象）
 */
struct Controller {
  Controller(PidTrajectoryTracker<6> &&ptt) : trajectory_tracker{std::move(ptt)} {}
  PidTrajectoryTracker<6> trajectory_tracker;
};
extern std::unique_ptr<Controller> controller;

namespace fsm {
namespace event {

struct Control {
  std::atomic<double> left_x;                       ///< 左摇杆x轴，范围[-1, 1]
  std::atomic<double> left_y;                       ///< 左摇杆y轴，范围[-1, 1]
  std::atomic<double> right_x;                      ///< 右摇杆x轴，范围[-1, 1]
  std::atomic<double> right_y;                      ///< 右摇杆y轴，范围[-1, 1]
  std::atomic<int> sa, sb, sc, sd, se, sf, sg, sh;  ///< 拨杆，上1中0下-1
  struct {
    std::atomic<bool> w, a, s, d, r, f, i, j, k, l;
  } keyboard_key;                           ///< 键盘按键
  std::atomic<uint16_t> keyboard_modifier;  ///< 键盘组合键modifier
};
template <typename T>
struct SwitchMode : tinyfsm::Event {};

}  // namespace event

class Arm : public tinyfsm::MooreMachine<Arm> {
 public:
  void react(tinyfsm::Event const &event);
  void react(event::Control const &control);

  template <typename Mode>
  void react(event::SwitchMode<Mode> const &) {
    if constexpr (std::is_base_of_v<Arm, Mode>) {
      if (this->is_in_state<Mode>()) {
        return;
      }
      transit<Mode>();
      info("MODE SWITCH: %s", abi::__cxa_demangle(typeid(Mode).name(), nullptr, nullptr, nullptr));
    }
  }
};

class GimbalBase : public tinyfsm::MooreMachine<GimbalBase> {
 public:
  void react(tinyfsm::Event const &event);
  void react(event::Control const &control);

  template <typename Mode>
  void react(event::SwitchMode<Mode> const &) {
    if constexpr (std::is_base_of_v<GimbalBase, Mode>) {
      if (this->is_in_state<Mode>()) {
        return;
      }
      transit<Mode>();
      info("MODE SWITCH: %s", abi::__cxa_demangle(typeid(Mode).name(), nullptr, nullptr, nullptr));
    }
  }
};

namespace state {
namespace arm {

/**
 * @brief 单关节调试
 */
class Debug : public fsm::Arm {
 public:
  void react(event::Control const &control);
  void entry();
};

/**
 * @brief 跟随参考轨迹
 */
class FollowTrajectory : public fsm::Arm {
 public:
  void react(event::Control const &control);
  void entry();
};

/**
 * @brief 机械臂保持收回
 */
class Retracted : public fsm::Arm {
 public:
  void react(event::Control const &control);
  void entry();
};

/**
 * @brief 机械臂无力
 */
class NoForce : public fsm::Arm {
 public:
  void react(event::Control const &control);
  void entry();
};

}  // namespace arm

namespace gimbal_base {

/**
 * @brief 大yaw轴跟底盘转
 */
class FollowChassis : public fsm::GimbalBase {
 public:
  void react(event::Control const &control);
  void entry();
};

/**
 * @brief 大yaw轴跟上层机械臂转
 *
 */
class FollowArm : public fsm::GimbalBase {
 public:
  void react(event::Control const &control);
  void entry();
};

/**
 * @brief 大yaw轴跟上层机械臂转，但是只有小yaw轴快碰到转动限位的时候才跟
 */
class FollowArmLazy : public fsm::GimbalBase {
 public:
  void react(event::Control const &control);
  void entry();
};

/**
 * @brief 大yaw轴无力
 */
class NoForce : public fsm::GimbalBase {
 public:
  void react(event::Control const &control);
  void entry();
};

}  // namespace gimbal_base

}  // namespace state

using fsm_list = tinyfsm::FsmList<Arm, GimbalBase>;

template <typename Event>
void DispatchEvent(Event const &event) {
  fsm_list::dispatch<Event>(event);
}

}  // namespace fsm

}  // namespace fluorite

#endif