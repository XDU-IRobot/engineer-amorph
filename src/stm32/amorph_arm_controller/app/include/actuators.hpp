#pragma once

#include <librm.hpp>

#include "arm_controller.hpp"
#include "encoder_counter.hpp"
#include "config.hpp"

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
        j6{upper_can, rm::device::DmMotorMit::Settings{.master_id = 0xb6,
                                                       .slave_id = 0xa6,
                                                       .p_max = 3.141593f,
                                                       .v_max = 30.f,
                                                       .t_max = 10.f,
                                                       .kp_range = {0, 500},
                                                       .kd_range = {0, 50}}},
        gripper{upper_can, CONFIG_GRIPPER_MOTOR_ID} {}
  void SetEnable(bool enable, int repeats = 5) {
    const auto dm_cmd = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
    for (int i = 0; i < repeats; ++i) {
      j1.SendInstruction(dm_cmd);
      j2.SendInstruction(dm_cmd);
      j3.SendInstruction(dm_cmd);
      {
        j4.SendInstruction(dm_cmd);
        j5.SendInstruction(dm_cmd);
        rm::Sleep(std::chrono::microseconds(900));
        j6.SendInstruction(dm_cmd);
        gripper.SetPower(0);
      }
    }
  }
  void ApplyOutput(const ArmController<7>::Output &output, bool apply_position = false) {
    // j4 j5 j6 夹爪四个电机的指令一次全发的话会把can tx buffer塞满，所以用这个标志来回切换做分时
    static bool upper_can_send_split = true;

    const auto &pd_config = CONFIG_MOTOR_PD_PARAMS;
    if (apply_position) {
      j1.SetMitCommand(calibrators[0].LogicalToReal(output.positions[0]),  //
                       0,                                                  //
                       0.f,                                                //
                       pd_config[0].kp,                                    //
                       pd_config[0].kd);
      j2.SetMitCommand(calibrators[1].LogicalToReal(output.positions[1]),                  //
                       0,                                                                  //
                       calibrators[1].reverse() ? -output.efforts[1] : output.efforts[1],  //
                       pd_config[1].kp,                                                    //
                       pd_config[1].kd);
      j3.SetMitCommand(calibrators[2].LogicalToReal(output.positions[2]),                  //
                       0,                                                                  //
                       calibrators[2].reverse() ? -output.efforts[2] : output.efforts[2],  //
                       pd_config[2].kp,                                                    //
                       pd_config[2].kd);
      if (upper_can_send_split) {
        j4.SetMitCommand(calibrators[3].LogicalToReal(output.positions[3]), 0, 0.f, pd_config[3].kp, pd_config[3].kd);
        j5.SetMitCommand(calibrators[4].LogicalToReal(output.positions[4]), 0, 0.f, pd_config[4].kp, pd_config[4].kd);
      } else {
        j6.SetMitCommand(calibrators[5].LogicalToReal(output.positions[5]), 0, 0.f, pd_config[5].kp, pd_config[5].kd);
        gripper.SetSpeed(output.velocities[6]);
      }
      // j1.SetMitCommand(calibrators[0].LogicalToReal(output.positions[0]), 0, 0.f, 0.f, 0.f);
      // j2.SetMitCommand(calibrators[1].LogicalToReal(output.positions[1]), 0,
      //                  calibrators[1].reverse() ? -output.efforts[1] : output.efforts[1], 0.f, 0.f);
      // calibrators[2].reverse() ? -output.efforts[2] : output.efforts[2], 0.f, 0.f);
      // if (upper_can_send_split) {
      // j4.SetMitCommand(calibrators[3].LogicalToReal(output.positions[3]), 0, 0.f, 0.f, 0.f);
      // j5.SetMitCommand(calibrators[4].LogicalToReal(output.positions[4]), 0, 0.f, 0.f, 0.f);
      // } else {
      // j6.SetMitCommand(calibrators[5].LogicalToReal(output.positions[5]), 0, 0.f, 0.f, 0.f);
      // gripper.SetSpeed(output.velocities[6]);
      // }
    } else {
      j1.SetMitCommand(0, 0, 0, 0, 0);
      j2.SetMitCommand(0, 0, 0, 0, 0);
      j3.SetMitCommand(0, 0, 0, 0, 0);
      if (upper_can_send_split) {
        j4.SetMitCommand(0, 0, 0, 0, 0);
        j5.SetMitCommand(0, 0, 0, 0, 0);
      } else {
        j6.SetMitCommand(0, 0, 0, 0, 0);
        gripper.SetPower(0);
      }
    }
    upper_can_send_split = !upper_can_send_split;
  }
  void UpdateEncoderCounter() { gripper_encoder.Update(gripper.feedback().position_rad); }

  /**
   * @brief 给达妙电机设零点
   */
  void DmSetZero() {
    SetEnable(true);
    for (int i = 0; i < 10; i++) {
      // j1.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      // j2.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      // j3.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      j4.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      // j5.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      // j6.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      HAL_Delay(10);
    }
    SetEnable(false);
  }

  std::array<float, 7> GetPositions() const {
    std::array<float, 7> positions{
        j1.pos(),                       //
        j2.pos(),                       //
        j3.pos(),                       //
        j4.pos(),                       //
        j5.pos(),                       //
        j6.pos(),                       //
        gripper_encoder.RealPosition()  //
    };
    // 对每个电机应用反转和零点偏移补偿
    for (int i = 0; i < 7; ++i) {
      positions[i] = calibrators[i].RealToLogical(positions[i]);
    }
    return positions;
  }

  std::array<float, 7> GetVelocities() const {
    std::array<float, 7> velocities{
        j1.vel(),                        //
        j2.vel(),                        //
        j3.vel(),                        //
        j4.vel(),                        //
        j5.vel(),                        //
        j6.vel(),                        //
        gripper.feedback().speed_rad_ps  //
    };

    // 对每个电机应用反转
    for (int i = 0; i < 7; ++i) {
      if (calibrators[i].reverse()) {
        velocities[i] = -velocities[i];
      }
    }

    return velocities;
  }

  std::array<float, 7> GetEfforts() const {
    std::array<float, 7> efforts{
        j1.tau(),                                     //
        j2.tau(),                                     //
        j3.tau(),                                     //
        j4.tau(),                                     //
        j5.tau(),                                     //
        j6.tau(),                                     //
        static_cast<float>(gripper.feedback().power)  //
    };

    // 对每个电机应用反转
    for (int i = 0; i < 7; ++i) {
      if (calibrators[i].reverse()) {
        efforts[i] = -efforts[i];
      }
    }

    return efforts;
  }

  rm::modules::MotorCalibrator calibrators[7]{CONFIG_MOTOR_CALIB_CONFIG};
  rm::device::DmMotorMit j1, j2, j3, j4, j5, j6;
  rm::device::LkMotorMS<15> gripper;
  EncoderCounter gripper_encoder{2.f * M_PI};
};