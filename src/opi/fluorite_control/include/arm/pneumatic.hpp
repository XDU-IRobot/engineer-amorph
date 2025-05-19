
#ifndef ARM_PNEUMATIC_HPP
#define ARM_PNEUMATIC_HPP

#include "arm/gpio.hpp"

namespace fluorite::arm {

/**
 * @brief 机械臂末端吸盘
 */
class SuckerSystem {
 public:
  SuckerSystem(OutputPin out_valve, OutputPin in_valve, PwmOutputPin vaccum_pump)
      : out_valve_{out_valve}, in_valve_{in_valve}, vaccum_pump_{vaccum_pump} {}

  enum class SuckerState { kSuck, kBlow, kOff };

  void Set(SuckerState state) {
    switch (state) {
      case SuckerState::kSuck:
        in_valve_.Write(HIGH);
        out_valve_.Write(HIGH);
        vaccum_pump_.Write(0.7);
        break;
      case SuckerState::kBlow:
        in_valve_.Write(LOW);
        out_valve_.Write(LOW);
        vaccum_pump_.Write(0.7);
        break;
      case SuckerState::kOff:
        in_valve_.Write(HIGH);
        out_valve_.Write(HIGH);
        vaccum_pump_.Write(0.0);
        break;
    }
    state_ = state;
  }

  void CycleMode() {
    switch (state_) {
      case SuckerState::kSuck:
        in_valve_.Write(LOW);
        out_valve_.Write(LOW);
        vaccum_pump_.Write(0.7);
        state_ = SuckerState::kBlow;
        break;
      case SuckerState::kBlow:
        in_valve_.Write(HIGH);
        out_valve_.Write(HIGH);
        vaccum_pump_.Write(0.0);
        state_ = SuckerState::kOff;
        break;
      case SuckerState::kOff:
        in_valve_.Write(HIGH);
        out_valve_.Write(HIGH);
        vaccum_pump_.Write(0.7);
        state_ = SuckerState::kSuck;
        break;
    }
  }

  auto state() const { return state_; }

 private:
  SuckerState state_{SuckerState::kOff};
  OutputPin out_valve_;
  OutputPin in_valve_;
  PwmOutputPin vaccum_pump_;
};

/**
 * @brief 存矿机构
 */
class StorageSystem {
 public:
  StorageSystem(OutputPin left_valve, OutputPin right_valve, PwmOutputPin vaccum_pump)
      : left_valve_{left_valve}, right_valve_{right_valve}, vaccum_pump_{vaccum_pump} {}

  /**
   * @brief 两侧电磁阀不通电，并关闭真空泵
   */
  void Disable() {
    left_valve_.Write(LOW);
    right_valve_.Write(LOW);
    vaccum_pump_.Write(0.0);
  }

  /**
   * @brief 设置左侧吸盘
   * @param state true将吸盘连接到真空泵，false封闭吸盘
   */
  void SetLeft(bool state) {
    left_valve_.Write(state);
    left_state_ = state;
    SetPumpPower();
  }

  void ToggleLeft() {
    left_state_ = !left_state_;
    left_valve_.Write(left_state_);
    SetPumpPower();
  }

  /**
   * @brief 设置右侧吸盘
   * @param state true将吸盘连接到真空泵，false封闭吸盘
   */
  void SetRight(bool state) {
    right_valve_.Write(state);
    right_state_ = state;
    SetPumpPower();
  }

  void ToggleRight() {
    right_state_ = !right_state_;
    right_valve_.Write(right_state_);
    SetPumpPower();
  }

  bool left_state() const { return left_state_; }
  bool right_state() const { return right_state_; }

 private:
  /**
   * @param power 真空泵功率，范围[0, 1]
   */
  void SetPumpPower() {
    if (left_state_ || right_state_) {
      vaccum_pump_.Write(0.7);
    } else {
      vaccum_pump_.Write(0.0);
    }
    // vaccum_pump_.Write(std::abs(power));
  }

  bool left_state_{false}, right_state_{false};
  OutputPin left_valve_;
  OutputPin right_valve_;
  PwmOutputPin vaccum_pump_;
};

}  // namespace fluorite::arm

#endif
