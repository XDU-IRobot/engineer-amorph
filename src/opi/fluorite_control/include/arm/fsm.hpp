/**
 * @file  fsm.hpp
 * @brief 使用tinyfsm库实现的状态机定义，分为大yaw轴底座和机械臂两个状态机，每个state类对应一种模式，control
 * event驱动控制逻辑运行，switchmode event驱动状态转移
 */

#ifndef ARM_FSM_HPP
#define ARM_FSM_HPP

#include <cstddef>
#include <tinyfsm.hpp>
#include <utility>

#include "keyboard_msgs/msg/key.hpp"
#include "fluorite_msgs/action/arm_end_effector_cartesian_command.hpp"
#include "fluorite_msgs/action/arm_joint_command.hpp"
#include "fluorite_msgs/action/arm_trajectory_command.hpp"
#include "fluorite_msgs/srv/storage_system_command.hpp"
#include "fluorite_msgs/srv/sucker_command.hpp"

#include "common/keypress_handler.hpp"
#include "common/action_server.hpp"
#include "arm/custom_controller.hpp"
#include "arm/global.hpp"

constexpr int kRcSwitchUp = 1;
constexpr int kRcSwitchMiddle = 0;
constexpr int kRcSwitchDown = -1;

namespace fluorite::arm {

namespace fsm {
namespace event {

/**
 * @brief 所有控制量全都在这里，其中的数据由ROS节点的各个回调函数负责更新
 */
struct Control : public tinyfsm::Event {
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
             keyboard_msgs::msg::Key::KEY_LSHIFT,
             keyboard_msgs::msg::Key::KEY_LCTRL,
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
  std::atomic<double> ld, rd;                       ///< 左右旋钮，范围[-1, 1]，逆负顺正
  std::atomic<double> ls, rs;                       ///< 遥控器背面的左右滑块，范围[-1, 1]，下负上正
  std::atomic<int> sa, sb, sc, sd, se, sf, sg, sh;  ///< 拨杆，上1中0下-1
  std::unordered_map<uint16_t, std::atomic<bool>> keyboard_key;  ///< 键盘按键

  bool arm_pump;
  bool left_storage;
  bool right_storage;

  custom_controller::JointCommand custom_controller_command;

  bool remote_failsafe{true};  ///< 遥控器失联标志
};
template <typename T>
struct SwitchMode : public tinyfsm::Event {};

/**
 * @brief   Action Server和Service Server接收到的来自行为树的命令，
            机械臂和气动系统在BehaviourTree模式下会且仅会采用这个控制量。
 * @details 这些控制量以及各自的active状态会在ActionServer的回调函数中被赋值，
 *          当有新的控制量到达时，active被置为true，执行完毕或者被取消后被置为false。
 *          后来的控制量会覆盖之前的控制量。当所有控制量的active都为false时，机械臂会保持在最后的状态不动。
 */
template <typename T>
struct BehaviourTreeCommand : public tinyfsm::Event {
  static_assert(std::is_same_v<T, fluorite_msgs::action::ArmEndEffectorCartesianCommand::Goal> ||
                    std::is_same_v<T, fluorite_msgs::action::ArmJointCommand::Goal> ||
                    std::is_same_v<T, fluorite_msgs::action::ArmTrajectoryCommand::Goal> ||
                    std::is_same_v<T, fluorite_msgs::srv::StorageSystemCommand::Request> ||
                    std::is_same_v<T, fluorite_msgs::srv::SuckerCommand::Request>,
                "BehaviourTreeCommand should only be one of ArmEndEffectorCartesianCommand, ArmJointCommand, "
                "ArmTrajectoryCommand, StorageSystemCommand and SuckerCommand");
  BehaviourTreeCommand(T command, std::function<void()> execution_finished_callback,
                       std::function<void()> execution_interrupted_callback)
      : command{command},
        execution_finished_callback{std::move(execution_finished_callback)},
        execution_interrupted_callback(std::move(execution_interrupted_callback)) {}
  T command;
  std::function<void()> execution_finished_callback, execution_interrupted_callback;
};
struct BehaviorTreeCancelExecution : public tinyfsm::Event {};

}  // namespace event

class Arm : public tinyfsm::MooreMachine<Arm> {
 public:
  virtual void entry() = 0;

  void react(tinyfsm::Event const &event);
  virtual void react(event::Control const &control) = 0;

  virtual void react(
      event::BehaviourTreeCommand<fluorite_msgs::action::ArmEndEffectorCartesianCommand::Goal> const &e) {}
  virtual void react(event::BehaviourTreeCommand<fluorite_msgs::action::ArmJointCommand::Goal> const &e) {}
  virtual void react(event::BehaviourTreeCommand<fluorite_msgs::action::ArmTrajectoryCommand::Goal> const &e) {}

  template <typename Mode>
  void react(event::SwitchMode<Mode> const &) {
    if constexpr (std::is_base_of_v<Arm, Mode>) {  // 首先判断这个模式属不属于这个状态机
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
    if constexpr (std::is_base_of_v<GimbalBase, Mode>) {  // 首先判断这个模式属不属于这个状态机
      if (this->is_in_state<Mode>()) {
        return;
      }
      info("MODE SWITCH: %s", abi::__cxa_demangle(typeid(Mode).name(), nullptr, nullptr, nullptr));
      transit<Mode>();
    }
  }
};

class PneumaticSystem : public tinyfsm::MooreMachine<PneumaticSystem> {
 public:
  virtual void entry() = 0;

  void react(tinyfsm::Event const &event);
  virtual void react(event::Control const &control) = 0;

  virtual void react(event::BehaviourTreeCommand<fluorite_msgs::srv::StorageSystemCommand::Request> const &e) {}
  virtual void react(event::BehaviourTreeCommand<fluorite_msgs::srv::SuckerCommand::Request> const &e) {}

  template <typename Mode>
  void react(event::SwitchMode<Mode> const &) {
    if constexpr (std::is_base_of_v<PneumaticSystem, Mode>) {  // 首先判断这个模式属不属于这个状态机
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

DeclareState(Debug, Arm);             ///< 单关节调试模式
DeclareState(KeyboardManual, Arm);    ///< 键鼠手动控制模式
DeclareState(CustomCtrlManual, Arm);  ///< 自定义控制器手动控制模式
// 行为树控制，空闲状态
class BehaviourTreeIdle : public Arm {
 public:
  void react(event::Control const &control) override;
  void react(
      event::BehaviourTreeCommand<fluorite_msgs::action::ArmEndEffectorCartesianCommand::Goal> const &e) override;
  void react(event::BehaviourTreeCommand<fluorite_msgs::action::ArmJointCommand::Goal> const &e) override;
  void react(event::BehaviourTreeCommand<fluorite_msgs::action::ArmTrajectoryCommand::Goal> const &e) override;
  void entry() override;
};
// 行为树控制，目前正在执行命令
class BehaviourTreeExecuting : public Arm {
  friend class BehaviourTreeIdle;

 public:
  void react(event::Control const &control) override;
  void react(event::BehaviorTreeCancelExecution const &e);
  void entry() override;

 protected:
  static std::function<void()> execution_finished_callback_;
  static std::function<void()> execution_interrupted_callback_;
};
DeclareState(Retracted, Arm);  ///< 机械臂保持收回
DeclareState(NoForce, Arm);    ///< 机械臂无力

}  // namespace arm

namespace gimbal_base {

DeclareState(FollowImu, GimbalBase);  ///< 大yaw轴有力
DeclareState(NoForce, GimbalBase);    ///< 大yaw轴无力

}  // namespace gimbal_base

namespace pneumatic_system {

DeclareState(BehaviourTree, PneumaticSystem);  ///< 气动系统行为树控制
DeclareState(Debug, PneumaticSystem);          ///< 气动系统手动控制
class Normal : public PneumaticSystem {
 public:
  void react(event::Control const &control) override;
  void react(event::BehaviourTreeCommand<fluorite_msgs::srv::StorageSystemCommand::Request> const &e) override;
  void react(event::BehaviourTreeCommand<fluorite_msgs::srv::SuckerCommand::Request> const &e) override;
  void entry() override;
};
DeclareState(Off, PneumaticSystem);  ///< 气动系统关闭

}  // namespace pneumatic_system

#undef DeclareState

}  // namespace state

using fsm_list = tinyfsm::FsmList<Arm, GimbalBase, PneumaticSystem>;

template <typename Event>
void DispatchEvent(Event const &event) {
  fsm_list::dispatch<Event>(event);
}

}  // namespace fsm

}  // namespace fluorite::arm

#endif