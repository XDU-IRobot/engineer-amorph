/**
 * @file  globals.hpp
 * @brief 向后兼容的全局资源访问头文件
 *
 * 原来的 Globals 单例已拆分为：
 *   - SharedResources  —— 全系统共享的硬件（IMU、RC、蜂鸣器、LED、通信CAN）
 *   - ArmInstance      —— 单个机械臂的全部控制对象（电机、规划器、FSM……）
 *
 * main.cc 直接使用 SharedResources 和 ArmInstance，无需通过此文件。
 * 此文件保留仅供未来可能的兼容性需求。
 */

#pragma once

#include "shared_resources.hpp"
#include "arm_instance.hpp"

