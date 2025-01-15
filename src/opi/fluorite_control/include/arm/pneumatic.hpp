
#include <arm/gpio.hpp>

/**
 * @brief 机械臂末端吸盘
 */
class SuckerSystem {
 public:
  SuckerSystem(OutputPin out_valve, OutputPin in_valve, PwmOutputPin vaccum_pump)
      : out_valve_{out_valve}, in_valve_{in_valve}, vaccum_pump_{vaccum_pump} {}

  /**
   * @brief 把真空泵的进出两端都连接到大气压，并关闭真空泵
   */
  void Disable() {
    in_valve_.Write(LOW);
    out_valve_.Write(HIGH);
    vaccum_pump_.Write(0.0);
  }

  /**
   * @brief 从吸盘吸气
   * @param power 吸气功率，范围[0, 1]，一般给0.7就可以吸住矿
   */
  void Suck(double power) {
    in_valve_.Write(HIGH);
    out_valve_.Write(HIGH);
    vaccum_pump_.Write(power);
  }

  /**
   * @brief 向吸盘吹气
   * @param power 吹气功率，范围[0, 1]
   */
  void Blow(double power) {
    in_valve_.Write(LOW);
    out_valve_.Write(LOW);
    vaccum_pump_.Write(power);
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
   * @brief 两侧电磁阀均通向大气压，并关闭真空泵
   */
  void Disable() {
    left_valve_.Write(HIGH);
    right_valve_.Write(HIGH);
    vaccum_pump_.Write(0.0);
  }

  /**
   * @param power 真空泵功率，范围[0, 1]
   */
  void SetPumpPower(double power) { vaccum_pump_.Write(power); }

  /**
   * @brief 设置左侧吸盘
   * @param state true将吸盘连接到真空泵，false将吸盘连接到大气压
   */
  void SetLeftValve(bool state) { left_valve_.Write(state); }

  /**
   * @brief 设置右侧吸盘
   * @param state true将吸盘连接到真空泵，false将吸盘连接到大气压
   */
  void SetRightValve(bool state) { right_valve_.Write(state); }

 private:
  OutputPin left_valve_;
  OutputPin right_valve_;
  PwmOutputPin vaccum_pump_;
};