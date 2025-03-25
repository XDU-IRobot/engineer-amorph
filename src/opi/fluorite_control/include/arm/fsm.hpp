/**
 * @file  fsm.hpp
 * @brief 使用tinyfsm库实现的状态机定义，分为大yaw轴底座和机械臂两个状态机，每个state类对应一种模式，control
 * event驱动控制逻辑运行，switchmode event驱动状态转移
 */

#ifndef ARM_FSM_HPP
#define ARM_FSM_HPP

#include <tinyfsm.hpp>

#include "arm/global.hpp"

namespace fluorite {

namespace fsm {
namespace event {

/**
 * @brief 所有链路的控制量全都在这里，数据由节点的各个subscriber回调函数负责更新
 */
struct Control : tinyfsm::Event {
  Control() : hotkeys{this} {
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

  KeypressHandler keypress_handler;
  struct _ {
    _(Control *this_) : this_(this_) {}
    Hotkey switch_arm_pump{[&]() -> bool {
      return this_->keyboard_key.at(keyboard_msgs::msg::Key::KEY_L) &&
             this_->keyboard_key.at(keyboard_msgs::msg::Key::KEY_LSHIFT);
    }};

   private:
    Control *this_;
  } hotkeys;
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

DeclareState(Debug, Arm);                     ///< 单关节调试模式
DeclareState(KeyboardManual, Arm);            ///< 键鼠手动控制模式
DeclareState(CustomCtrlManual, Arm);          ///< 自定义控制器手动控制模式
DeclareState(FollowExternalTrajectory, Arm);  ///< 跟随Action Server输入的参考轨迹
DeclareState(Retracted, Arm);                 ///< 机械臂保持收回
DeclareState(NoForce, Arm);                   ///< 机械臂无力

}  // namespace arm

namespace gimbal_base {

DeclareState(FollowImu, GimbalBase);  ///< 大yaw轴有力
DeclareState(NoForce, GimbalBase);    ///< 大yaw轴无力

}  // namespace gimbal_base

namespace pneumatic_system {

DeclareState(On, PneumaticSystem);   ///< 气泵系统有力
DeclareState(Off, PneumaticSystem);  ///< 气泵系统无力

}  // namespace pneumatic_system

#undef DeclareState

}  // namespace state

using fsm_list = tinyfsm::FsmList<Arm, GimbalBase, PneumaticSystem>;

template <typename Event>
void DispatchEvent(Event const &event) {
  fsm_list::dispatch<Event>(event);
}

}  // namespace fsm

}  // namespace fluorite

#endif