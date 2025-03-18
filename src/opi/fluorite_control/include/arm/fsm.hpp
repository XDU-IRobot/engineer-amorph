/**
 * @file  fsm.hpp
 * @brief 使用tinyfsm库实现的状态机定义，分为大yaw轴底座和机械臂两个状态机，每个state类对应一种模式，control
 * event驱动控制逻辑运行，switchmode event驱动状态转移
 */

#ifndef ARM_FSM_HPP
#define ARM_FSM_HPP

#include <mutex>
#include <atomic>
#include <memory>
#include <type_traits>

#include <tinyfsm.hpp>
#include <librm.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include "keyboard_msgs/msg/key.hpp"

#include "fluorite_msgs/msg/arm_state.hpp"
#include "arm/pneumatic.hpp"
#include "arm/pid_trajectory_tracker.hpp"
#include "common/thread_pool.hpp"
#include "common/logger.hpp"

// 使用tinyfsm之后，就没办法再使用oop的各种编程范式来写代码了，因为状态机类没有对象
// 这里被迫创建了几个全局变量（定义都在fsm.cc里），用来存储状态机的外部依赖
// 如果以后找到了更好的实现方式，可以考虑重构
namespace fluorite {

/**
 * @brief   机器人状态向量，由节点的各个传感器数据回调函数更新
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
  using PosPID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  Controller(PidTrajectoryTracker<6> &&ptt, PosPID &&base_yaw_speed_pid, PosPID &&base_yaw_position_pid,
             moveit::core::RobotStatePtr &&kinematic_model, moveit::planning_interface::MoveGroupInterface &&move_group)
      : trajectory_tracker{std::move(ptt)},
        base_yaw_speed_pid{std::move(base_yaw_speed_pid)},
        base_yaw_position_pid{std::move(base_yaw_position_pid)},
        kinematic_model(std::move(kinematic_model)),
        move_group(std::move(move_group)) {}
  PidTrajectoryTracker<6> trajectory_tracker;
  PosPID base_yaw_speed_pid, base_yaw_position_pid;

  moveit::core::RobotStatePtr kinematic_model;
  moveit::planning_interface::MoveGroupInterface move_group;
  std::mutex move_group_request_mutex;
  ThreadPool move_group_request_thread_pool_{1};
};
extern std::unique_ptr<Controller> controller;

namespace fsm {
namespace event {

struct Control : tinyfsm::Event {
  Control() {
    // map里的std::atomic<bool>不能用std::initializer_list初始化，必须一个一个构造
    for (const auto &keyboard_map_k : {
             keyboard_msgs::msg::Key::KEY_W,
             keyboard_msgs::msg::Key::KEY_A,
             keyboard_msgs::msg::Key::KEY_S,
             keyboard_msgs::msg::Key::KEY_D,
             keyboard_msgs::msg::Key::KEY_R,
             keyboard_msgs::msg::Key::KEY_F,
             keyboard_msgs::msg::Key::KEY_I,
             keyboard_msgs::msg::Key::KEY_J,
             keyboard_msgs::msg::Key::KEY_K,
             keyboard_msgs::msg::Key::KEY_L,
             keyboard_msgs::msg::Key::KEY_U,
             keyboard_msgs::msg::Key::KEY_O,
             keyboard_msgs::msg::Key::KEY_G,
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
  std::atomic<int> sa, sb, sc, sd, se, sf, sg, sh;  ///< 拨杆，上1中0下-1
  std::unordered_map<uint16_t, bool> keyboard_key;  ///< 键盘按键
  std::atomic<uint16_t> keyboard_modifier;          ///< 键盘组合键modifier
};
template <typename T>
struct SwitchMode : tinyfsm::Event {};

}  // namespace event

class Arm : public tinyfsm::MooreMachine<Arm> {
 public:
  virtual void entry() = 0;

  void react(tinyfsm::Event const &event);
  virtual void react(event::Control const &control) = 0;

  template <typename Mode>
  void react(event::SwitchMode<Mode> const &) {
    if constexpr (std::is_base_of_v<Arm, Mode>) {  // 首先要判断这个模式属不属于这个状态机
      if (this->is_in_state<Mode>()) {
        return;
      }
      info("MODE SWITCH: %s", abi::__cxa_demangle(typeid(Mode).name(), nullptr, nullptr, nullptr));
      transit<Mode>();
    }
  }

 protected:
  static void EnableActuators(bool enable);
  static void CalcPidAndExecute();
};

class GimbalBase : public tinyfsm::MooreMachine<GimbalBase> {
 public:
  virtual void entry() = 0;

  void react(tinyfsm::Event const &event);
  virtual void react(event::Control const &control) = 0;

  template <typename Mode>
  void react(event::SwitchMode<Mode> const &) {
    if constexpr (std::is_base_of_v<GimbalBase, Mode>) {  // 首先要判断这个模式属不属于这个状态机
      if (this->is_in_state<Mode>()) {
        return;
      }
      info("MODE SWITCH: %s", abi::__cxa_demangle(typeid(Mode).name(), nullptr, nullptr, nullptr));
      transit<Mode>();
    }
  }
};

/**
 * @brief 机械臂的各种运行模式
 */
namespace state {

#define DeclareState(state_name, fsm)                                                                 \
  class state_name : public fsm {                                                                     \
    static_assert(std::is_base_of_v<tinyfsm::Fsm<fsm>, fsm>, "Given fsm class is not tinyfsm::Fsm!"); \
                                                                                                      \
   public:                                                                                            \
    void react(event::Control const &control) override;                                               \
    void entry() override;                                                                            \
  }

namespace arm {

/**
 * @brief 单关节调试模式
 */
DeclareState(Debug, Arm);

/**
 * @brief 键鼠手动控制模式
 */
DeclareState(KeyboardManual, Arm);

/**
 * @brief 自定义控制器手动控制模式
 */
DeclareState(CustomCtrlManual, Arm);

/**
 * @brief 跟随Action Server输入的参考轨迹
 */
DeclareState(FollowExternalTrajectory, Arm);

/**
 * @brief 机械臂保持收回
 */
DeclareState(Retracted, Arm);

/**
 * @brief 机械臂无力
 */
DeclareState(NoForce, Arm);

}  // namespace arm

namespace gimbal_base {

/**
 * @brief 大yaw轴跟上层机械臂转
 *
 */
DeclareState(FollowArm, GimbalBase);

/**
 * @brief 大yaw轴跟上层机械臂转，但是只有小yaw轴快碰到转动限位的时候才跟
 */
class FollowArmLazy : public fsm::GimbalBase {
 public:
  void react(event::Control const &control);
  void entry();

 private:
  enum class LazyState { kStay, kFollow };

 private:
  static double lazily_stay_at_;
  static LazyState lazy_state_;
};

/**
 * @brief 大yaw轴无力
 */
DeclareState(NoForce, GimbalBase);

}  // namespace gimbal_base

#undef DeclareState

}  // namespace state

using fsm_list = tinyfsm::FsmList<Arm, GimbalBase>;

template <typename Event>
void DispatchEvent(Event const &event) {
  fsm_list::dispatch<Event>(event);
}

}  // namespace fsm

}  // namespace fluorite

#endif