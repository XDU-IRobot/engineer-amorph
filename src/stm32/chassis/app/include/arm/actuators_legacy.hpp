#pragma once

#include <librm.hpp>

#include "arm_controller.hpp"
#include "config.hpp"
#include "encoder_counter.hpp"

/**
 * @brief 用来储存所有电机，以及实现一些电机相关的helper（使能、处理反转、设零点）
 *
 * 原来通过编译期宏注入的参数（CAN总线、电机ID、校准配置、PD增益）现在全部
 * 通过 ArmConfig 在运行期传入，从而支持一个固件中创建多个 ActuatorLegacy 实例。
 */
class ActuatorLegacy {
 public:
  explicit ActuatorLegacy(const ArmConfig& config)
      : j1{*config.can, config.dm_settings[0]},
        j2{*config.can, config.dm_settings[1]},
        j3{*config.can, config.dm_settings[2]},
        j4{*config.can, config.dm_settings[3]},
        j5{*config.can, config.dm_settings[4]},
        j6{*config.can, config.j6_motor_id},
        gripper{*config.can, config.gripper_motor_id} {
    for (int i = 0; i < 7; ++i) {
      calibrators[i].UpdateConfig(config.calibrators[i]);
    }
    for (int i = 0; i < 6; ++i) {
      pd_params_[i] = config.pd_params[i];
    }
  }

  void SetEnable(bool enable, int repeats = 2) {
    const auto dm_cmd = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
    for (int i = 0; i < repeats; ++i) {
      j1.SendInstruction(dm_cmd);
      j2.SendInstruction(dm_cmd);
      j3.SendInstruction(dm_cmd);
      j4.SendInstruction(dm_cmd);
      j5.SendInstruction(dm_cmd);
      j6.SetCurrent(0);
      gripper.SetPower(0);
    }
  }

  void ApplyOutput(const ArmController::Output& output, bool apply_position = false) {
    const auto& pd_config = pd_params_;
    if (apply_position) {
      j1.SetMitCommand(calibrators[0].LogicalToReal(output.positions[0]),                        //
                       calibrators[0].reverse() ? -output.velocities[0] : output.velocities[0],  //
                       calibrators[0].reverse() ? -output.efforts[0] : output.efforts[0],        //
                       pd_config[0].kp,                                                          //
                       pd_config[0].kd);
      j2.SetMitCommand(calibrators[1].LogicalToReal(output.positions[1]),                        //
                       calibrators[1].reverse() ? -output.velocities[1] : output.velocities[1],  //
                       calibrators[1].reverse() ? -output.efforts[1] : output.efforts[1],        //
                       pd_config[1].kp,                                                          //
                       pd_config[1].kd);
      j3.SetMitCommand(calibrators[2].LogicalToReal(output.positions[2]) * 2.f,                  // 1:1.5减速比
                       calibrators[2].reverse() ? -output.velocities[2] : output.velocities[2],  //
                       calibrators[2].reverse() ? -output.efforts[2] : output.efforts[2],        //
                       pd_config[2].kp,                                                          //
                       pd_config[2].kd);
      j4.SetMitCommand(calibrators[3].LogicalToReal(output.positions[3]),  //
                       0,                                                  //
                       0.f,                                                //
                       pd_config[3].kp,                                    //
                       pd_config[3].kd);
      j5.SetMitCommand(calibrators[4].LogicalToReal(output.positions[4]),  //
                       0,                                                  //
                       0.f,                                                //
                       pd_config[4].kp,                                    //
                       pd_config[4].kd);
      j6.SetCurrent(calibrators[5].reverse() ? -output.efforts[5] : output.efforts[5]);
    } else {
      j1.SetMitCommand(0, 0, 0, 0, 0);
      j2.SetMitCommand(0, 0, 0, 0, 0);
      j3.SetMitCommand(0, 0, 0, 0, 0);
      j4.SetMitCommand(0, 0, 0, 0, 0);
      j5.SetMitCommand(0, 0, 0, 0, 0);
      j6.SetCurrent(0);
    }
    // j1.SetMitCommand(0, 0, 0, 0, 0);
    // j2.SetMitCommand(0, 0, 0, 0, 0);
    // j3.SetMitCommand(0, 0, 0, 0, 0);
    // j4.SetMitCommand(0, 0, 0, 0, 0);
    // j5.SetMitCommand(0, 0, 0, 0, 0);
    // j6.SetCurrent(0);
  }

  void QueryEncoders() {
    gripper.SendInstruction(rm::device::LkMotor<rm::device::LkMotorType::kMS, 15>::kReadPositionMulti);
    gripper_encoder.Update(gripper.feedback().position_rad);
    j6_encoder.Update(calibrators[5].RealToLogical(j6.pos_rad()));
  }

  /**
   * @brief 给达妙电机设零点
   */
  void DmSetZero() {
    SetEnable(true, 2);
    for (int i = 0; i < 2; i++) {
      // j1.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      // j2.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      j3.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      // j4.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      // j5.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
    }
    SetEnable(false, 2);
  }

  std::array<float, 6> GetPositions() const {
    std::array<float, 6> positions{
        j1.pos(),                   //
        j2.pos(),                   //
        j3.pos() / 2.f,             // 1:1.5减速比
        j4.pos(),                   //
        j5.pos(),                   //
        j6_encoder.RealPosition(),  //
    };
    for (int i = 0; i < 5; ++i) {
      positions[i] = calibrators[i].RealToLogical(positions[i]);
    }
    return positions;
  }

  float GetGripperPosition() const { return calibrators[6].RealToLogical(gripper.feedback().position_multi_rad); }

  float GetGripperVelocity() const {
    const float vel = gripper.feedback().speed_rad_ps;
    return calibrators[6].reverse() ? -vel : vel;
  }

  float GetGripperPower() const { return gripper.feedback().power; }

  std::array<float, 6> GetVelocities() const {
    std::array<float, 6> velocities{
        j1.vel(),                       //
        j2.vel(),                       //
        j3.vel(),                       //
        j4.vel(),                       //
        j5.vel(),                       //
        j6.rpm() * 2.f * M_PIf / 60.f,  //
    };
    for (int i = 0; i < 6; ++i) {
      if (calibrators[i].reverse()) {
        velocities[i] = -velocities[i];
      }
    }
    return velocities;
  }

  std::array<float, 6> GetEfforts() const {
    std::array<float, 6> efforts{
        j1.tau(),                          //
        j2.tau(),                          //
        j3.tau(),                          //
        j4.tau(),                          //
        j5.tau(),                          //
        static_cast<float>(j6.current()),  //
    };
    for (int i = 0; i < 6; ++i) {
      if (calibrators[i].reverse()) {
        efforts[i] = -efforts[i];
      }
    }
    return efforts;
  }

  // 校准器保持 public，供 GripperController 校准完成回调中更新零点
  rm::modules::MotorCalibrator calibrators[7];
  rm::device::DmMotorMit j1, j2, j3, j4, j5;
  rm::device::GM6020 j6;
  rm::device::LkMotorMS<15> gripper;
  EncoderCounter gripper_encoder{2.f * M_PIf};
  EncoderCounter j6_encoder{2.f * M_PIf};

 private:
  ArmConfig::PDParams pd_params_[6]{};  ///< 从 ArmConfig 拷贝的 PD 参数
  float j3_spring_k_{0.01f};
};