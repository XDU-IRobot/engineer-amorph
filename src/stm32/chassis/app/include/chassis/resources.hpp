#pragma once

#include <librm.hpp>

#include "config.hpp"
#include "shared_resources.hpp"
#include "tri_steer_chassis_controller.hpp"

struct ChassisResources {
  explicit ChassisResources(const ChassisConfig &config)
      : f_steer{SharedResources::GetInstance().mcp_can, config.f_steer_id},  //
        l_steer{SharedResources::GetInstance().mcp_can, config.l_steer_id},
        r_steer{SharedResources::GetInstance().mcp_can, config.r_steer_id},
        f_wheel{SharedResources::GetInstance().mcp_can, config.f_wheel_id},
        l_wheel{SharedResources::GetInstance().mcp_can, config.l_wheel_id},
        r_wheel{SharedResources::GetInstance().mcp_can, config.r_wheel_id},
        f_steer_calib{config.f_steer_calib},
        l_steer_calib{config.l_steer_calib},
        r_steer_calib{config.r_steer_calib},
        controller{config} {
    f_steer.SetName("f_steer");
    l_steer.SetName("l_steer");
    r_steer.SetName("r_steer");
    f_wheel.SetName("f_wheel");
    l_wheel.SetName("l_wheel");
    r_wheel.SetName("r_wheel");
    device_manager << &f_steer << &l_steer << &r_steer << &f_wheel << &l_wheel << &r_wheel;
  }

  rm::device::GM6020Current f_steer, l_steer, r_steer;                       ///< 舵电机
  rm::device::M3508 f_wheel, l_wheel, r_wheel;                               ///< 轮电机
  rm::modules::MotorCalibrator f_steer_calib, l_steer_calib, r_steer_calib;  ///< 舵电机校准器

  TriSteerChassisController controller;  ///< 轨迹规划 + 控制输出计算

  rm::device::DeviceManager<10> device_manager;  ///< 管理所有电机的在线状态
};