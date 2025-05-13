
#ifndef ARM_PNEUMATIC_HPP
#define ARM_PNEUMATIC_HPP

#include "arm/gpio.hpp"

/**
 * @brief 机械臂末端吸盘
 */
class SuckerSystem {
 public:
  SuckerSystem(OutputPin out_valve, OutputPin in_valve, PwmOutputPin vaccum_pump)
      : out_valve_{out_valve}, in_valve_{in_valve}, vaccum_pump_{vaccum_pump} {}

  /**
   * @brief 两侧电磁阀不通电，并关闭真空泵
   */
  void Disable() {
    in_valve_.Write(HIGH);
    out_valve_.Write(HIGH);
    vaccum_pump_.Write(0.0);
  }

  /**
   * @brief 从吸盘吸气/吹气
   * @param power 真空泵功率，范围[-1, 1]，正数为吸气，负数为吹气，一般给0.7就可以吸住矿
   */
  void Suck(double power) {
    if (power > 0) {
      in_valve_.Write(HIGH);
      out_valve_.Write(HIGH);
    } else {
      in_valve_.Write(LOW);
      out_valve_.Write(LOW);
    }
    vaccum_pump_.Write(std::abs(power));
  }

 private:
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
   * @param power 真空泵功率，范围[0, 1]
   */
  void SetPumpPower() {
    if (left_state_ || right_state_) {
      vaccum_pump_.Write(1.0);
    } else {
      vaccum_pump_.Write(0.0);
    }
    // vaccum_pump_.Write(std::abs(power));
  }

  /**
   * @brief 设置左侧吸盘
   * @param state true将吸盘连接到真空泵，false封闭吸盘
   */
  void SetLeftValve(bool state) {
    left_valve_.Write(state);
    left_state_ = state;
  }

  /**
   * @brief 设置右侧吸盘
   * @param state true将吸盘连接到真空泵，false封闭吸盘
   */
  void SetRightValve(bool state) {
    right_valve_.Write(state);
    right_state_ = state;
  }

 private:
  bool left_state_{false}, right_state_{false};
  OutputPin left_valve_;
  OutputPin right_valve_;
  PwmOutputPin vaccum_pump_;
};

#endif
