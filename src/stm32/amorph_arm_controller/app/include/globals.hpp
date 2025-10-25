
#pragma once

#include <librm.hpp>

#include "fdcan.h"
#include "usart.h"

#include "fsm.hpp"
#include "mpc_solver.hpp"
#include "dynamics.hpp"
#include "rpc/rpc.hpp"

/**
 * @brief   全局数据仓库
 * @note    能封装的尽量封装、能依赖注入的尽量依赖注入，这里出现的全局变量越少越好，保持数据流向清晰
 */
struct Globals {
  rm::hal::Can motor_can1{hfdcan1}, motor_can2{hfdcan2}, rpc_can{hfdcan3};  ///< 用于电机和RPC通信的CAN总线
  rm::hal::Serial sbus_serial{huart5, 50, rm::hal::stm32::UartMode::kNormal,
                              rm::hal::stm32::UartMode::kDma};  ///< 用于接收遥控器信号的串口
  RPC rpc;                                                      ///< RPC对象，负责与上位机通信
  rm::device::Sbus rc{sbus_serial};                             ///< 遥控器
  fsm::Arm fsm;                                                 ///< 机械臂的有限状态机
  DynamicModel dynamic_model;                                   ///< 机械臂的动力学模型
  MpcSolver mpc_solver;                                         ///< MPC求解器
  struct {
    using DmMit = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;
    // TODO
  } actuator;
};

extern Globals *globals;