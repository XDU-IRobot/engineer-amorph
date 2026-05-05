/**
 * @file  chassis/chassis.hpp
 * @brief 单个底盘实例的完整封装
 */

#pragma once

#include "config.hpp"
#include "resources.hpp"
#include "fsm.hpp"

/**
 * @brief 单个底盘实例，持有底盘相关的资源和状态机实例
 */
struct Chassis {
  ChassisResources resources;

  // 状态机
  fsm_chassis::Chassis fsm{resources};
  fsm_chassis::state::NoForce fsm_state_no_force;
  fsm_chassis::state::Normal fsm_state_normal;
  fsm_chassis::state::Fault fsm_state_idle;

  float measured_power{0.f};

 private:
  // 这个数组需与fsm同生命周期，所以放在这里
  etl::ifsm_state *state_list_[3]{
      &fsm_state_no_force,
      &fsm_state_normal,
      &fsm_state_idle,
  };

 public:
  explicit Chassis(const ChassisConfig &config) : resources{config} {}

  /**
   * @brief 初始化（设备命名、回调注册）
   */
  void Init() {
    auto &shared = SharedResources::GetInstance();

    // 定义遥控器拨杆功能
    shared.rc(rc_ch::SD).OnValueChange([this](const int &old_value, const int &new_value) {
      // 353-up, 1024-mid, 1695-down
      if (old_value == 0) {
        return;  // 第一次触发不响应
      }
      if (new_value > 1300 && new_value < 1700) {  // SD在下
        fsm.receive(fsm_chassis::event::FaultClear{});
        fsm.receive(fsm_chassis::event::ForceModeSwitch{fsm_chassis::StateId::kNoForce});
      }
      if (new_value > 0 && new_value < 500) {  // SD在上
        fsm.receive(fsm_chassis::event::ForceModeSwitch{fsm_chassis::StateId::kNormal});
      }
      if (new_value > 500 && new_value < 1100) {  // SD在中
        fsm.receive(fsm_chassis::event::ForceModeSwitch{fsm_chassis::StateId::kNoForce});
      }
    });

    shared.device_manager.OnDeviceStatusChange([&](rm::device::Device *dev) {
      if (dev->name() == "remote" && dev->online_status() != rm::device::Device::kOk) {  // 遥控器离线，无力
        fsm.receive(fsm_chassis::event::ForceModeSwitch{fsm_chassis::StateId::kNoForce});
      }
    });

    fsm.set_states(state_list_, std::size(state_list_));
    fsm.start();
  }

  /**
   * @brief 500 Hz 控制循环节拍（IMU 数据从 SharedResources 内部读取）
   * @note  调用前须已调用 SharedResources::imu.Update()
   */
  void Tick() {
    resources.device_manager.Update();
    const float motor_current_sum =            //
        (resources.f_steer.current() * 3.f +   //
         resources.l_steer.current() * 3.f +   //
         resources.r_steer.current() * 3.f +   //
         resources.f_wheel.current() * 20.f +  //
         resources.l_wheel.current() * 20.f +  //
         resources.r_wheel.current() * 20.f) /
        16384.f;
    measured_power = motor_current_sum * SharedResources::GetInstance().vbus;
    const fsm_chassis::event::ControlLoop control_loop_event = [this] {
      using rm::modules::Wrap;
      fsm_chassis::event::ControlLoop ret;
      ret.f_steer_pos = Wrap(resources.f_steer_calib.RealToLogical(resources.f_steer.pos_rad()), 0, M_PI * 2.f);
      ret.f_steer_vel = -resources.f_steer.rpm() / 60.f * 2.f * M_PI;  // to rad/s
      ret.l_steer_pos = Wrap(resources.l_steer_calib.RealToLogical(resources.l_steer.pos_rad()), 0, M_PI * 2.f);
      ret.l_steer_vel = -resources.l_steer.rpm() / 60.f * 2.f * M_PI;
      ret.r_steer_pos = Wrap(resources.r_steer_calib.RealToLogical(resources.r_steer.pos_rad()), 0, M_PI * 2.f);
      ret.r_steer_vel = -resources.r_steer.rpm() / 60.f * 2.f * M_PI;
      ret.f_wheel_vel = -resources.f_wheel.rpm() / 60.f * 2.f * M_PI;
      ret.l_wheel_vel = -resources.l_wheel.rpm() / 60.f * 2.f * M_PI;
      ret.r_wheel_vel = -resources.r_wheel.rpm() / 60.f * 2.f * M_PI;
      ret.f_wheel_current = -resources.f_wheel.current();
      ret.l_wheel_current = -resources.l_wheel.current();
      ret.r_wheel_current = -resources.r_wheel.current();
      return ret;
    }();
    fsm.receive(control_loop_event);
    rm::device::DjiMotorBase::SendCommand(SharedResources::GetInstance().mcp_can);
  }
};