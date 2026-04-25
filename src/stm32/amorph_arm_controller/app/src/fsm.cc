/**
 * @file  fsm.cc
 * @brief FSM 实例和状态对象现由 ArmInstance 持有，本文件仅保留为编译占位。
 *
 * 原来的全局 fsm::arm 和 namespace state 下的文件级静态状态实例已移除：
 *   - fsm::Arm arm            → ArmInstance::arm_fsm
 *   - state::NoForce no_force → ArmInstance::state_no_force
 *   - ... 以此类推
 *
 * 原来的 fsm::Init()          → 在 ArmInstance 构造函数中完成
 */

#include "fsm.hpp"
