#pragma once

#include <librm.hpp>

#include "arm_controller.hpp"
#include "encoder_counter.hpp"

/**
 * @brief 用来储存所有电机，以及实现一些电机相关的helper（使能、处理反转、设零点）
 */
class Actuator {
 public:
  Actuator(rm::hal::Can &lower_can, rm::hal::Can &upper_can)
      : j1{lower_can, rm::device::DmMotorMit::Settings{.master_id = 0xb1,
                                                       .slave_id = 0xa1,
                                                       .p_max = 3.141593f,
                                                       .v_max = 20.f,
                                                       .t_max = 28.f,
                                                       .kp_range = {0, 500},
                                                       .kd_range = {0, 50}}},
        j2{lower_can, rm::device::DmMotorMit::Settings{.master_id = 0xb2,
                                                       .slave_id = 0xa2,
                                                       .p_max = 3.141593f,
                                                       .v_max = 45.f,
                                                       .t_max = 54.f,
                                                       .kp_range = {0, 500},
                                                       .kd_range = {0, 50}}},
        j3{lower_can, rm::device::DmMotorMit::Settings{.master_id = 0xb3,
                                                       .slave_id = 0xa3,
                                                       .p_max = 3.141593f,
                                                       .v_max = 20.f,
                                                       .t_max = 28.f,
                                                       .kp_range = {0, 500},
                                                       .kd_range = {0, 50}}},
        j4{upper_can, rm::device::DmMotorMit::Settings{.master_id = 0xb4,
                                                       .slave_id = 0xa4,
                                                       .p_max = 3.141593f,
                                                       .v_max = 30.f,
                                                       .t_max = 10.f,
                                                       .kp_range = {0, 500},
                                                       .kd_range = {0, 50}}},
        j5{upper_can, rm::device::DmMotorMit::Settings{.master_id = 0xb5,
                                                       .slave_id = 0xa5,
                                                       .p_max = 3.141593f,
                                                       .v_max = 30.f,
                                                       .t_max = 10.f,
                                                       .kp_range = {0, 500},
                                                       .kd_range = {0, 50}}},
        j6{upper_can, 2},
        gripper{upper_can, 1} {}
  void SetEnable(bool enable, int repeats = 5) {
    const auto cmd = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
    for (int i = 0; i < repeats; ++i) {
      j1.SendInstruction(cmd);
      j2.SendInstruction(cmd);
      j3.SendInstruction(cmd);
      j4.SendInstruction(cmd);
      j5.SendInstruction(cmd);
      // NOTE: 3508和2006有越转越快的bug，需要在NoForce状态里面处理减速，直接发0电流没有作用
      // if (!enable) {
      //   gripper->SetCurrent(0);
      //   j6->SetCurrent(0);
      //   rm::device::DjiMotorBase::SendCommand();
      // }
    }
  }
  void ApplyOutput(const ArmController<7>::Output &output, bool apply_position = false) {
    if (apply_position) {
      j1.SetMitCommand(calibrators[0].LogicalToReal(output.positions[0]), 0, 0.f, 50.f, 25.f);
      j2.SetMitCommand(calibrators[1].LogicalToReal(output.positions[1]), 0,
                       calibrators[1].reverse() ? -output.efforts[1] : output.efforts[1], 80.f, 30.f);
      j3.SetMitCommand(calibrators[2].LogicalToReal(output.positions[2]), 0,
                       calibrators[2].reverse() ? -output.efforts[2] : output.efforts[2], 50.f, 30.f);
      j4.SetMitCommand(calibrators[3].LogicalToReal(output.positions[3]), 0, 0.f, 35.f, 15.f);
      j5.SetMitCommand(calibrators[4].LogicalToReal(output.positions[4]), 0, 0.f, 35.f, 15.f);
    } else {
      j1.SetMitCommand(0, 0, 0, 0, 0);
      j2.SetMitCommand(0, 0, 0, 0, 0);
      j3.SetMitCommand(0, 0, 0, 0, 0);
      j4.SetMitCommand(0, 0, 0, 0, 0);
      j5.SetMitCommand(0, 0, 0, 0, 0);
    }
    // j1.SetMitCommand(0, 0, 0, 0, 0);
    // j2.SetMitCommand(0, 0, 0, 0, 0);
    // j3.SetMitCommand(0, 0, 0, 0, 0);
    // j4.SetMitCommand(0, 0, 0, 0, 0);
    // j5.SetMitCommand(0, 0, 0, 0, 0);
    j6.SetCurrent(0);
    gripper.SetCurrent(0);
    rm::device::DjiMotorBase::SendCommand();
  }
  void UpdateEncoderCounter() {
    j6_encoder.Update(j6.pos_rad());
    gripper_encoder.Update(gripper.pos_rad());
  }

  /**
   * @brief 给达妙电机设零点
   */
  void DmSetZero() {
    SetEnable(true);
    for (int i = 0; i < 10; i++) {
      // j1.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      j2.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      j3.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      // j4.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      // j5.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      HAL_Delay(10);
    }
    SetEnable(false);
  }

  std::array<float, 7> GetPositions() {
    std::array<float, 7> positions{
        j1.pos(),                       //
        j2.pos(),                       //
        j3.pos(),                       //
        j4.pos(),                       //
        j5.pos(),                       //
        j6_encoder.RealPosition(),      //
        gripper_encoder.RealPosition()  //
    };
    // 对每个电机应用反转和零点偏移补偿
    for (int i = 0; i < 7; ++i) {
      positions[i] = calibrators[i].RealToLogical(positions[i]);
    }

    // 处理2006的输出轴减速比，2006减速比1:36，齿轮传动1:1
    positions[5] /= 36.f;
    positions[6] /= 36.f;

    return positions;
  }

  std::array<float, 7> GetVelocities() const {
    // 从编码器读取原始速度
    std::array<float, 7> velocities{
        j1.vel(),
        j2.vel(),
        j3.vel(),
        j4.vel(),
        j5.vel(),
        j6.rpm() / 60.f * 2.f * std::numbers::pi_v<float> / 36.f,
        gripper.rpm() / 60.f * 2.f * std::numbers::pi_v<float> / 36.f  // 2006减速比1:36，齿轮传动1:1
    };

    // 对每个电机应用反转
    for (int i = 0; i < 7; ++i) {
      if (calibrators[i].reverse()) {
        velocities[i] = -velocities[i];
      }
    }

    return velocities;
  }

  rm::modules::MotorCalibrator calibrators[7]{{true, 0.f},                                                 //
                                              {false, -rm::modules::Angle::FromDeg(90.f - 11.93f).rad()},  //
                                              {false, rm::modules::Angle::FromDeg(40.f - 11.93f).rad()},   //
                                              {false, 0.f},                                                //
                                              {true, 0.f},                                                 //
                                              {true, 0.f},                                                 //
                                              {false, 0.f}};
  rm::device::DmMotorMit j1, j2, j3, j4, j5;
  rm::device::M2006 j6, gripper;
  EncoderCounter j6_encoder{2.f * M_PI}, gripper_encoder{2.f * M_PI};  // 2006减速比1:36
};