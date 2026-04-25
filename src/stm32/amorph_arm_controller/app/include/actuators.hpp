#pragma once

#include <librm.hpp>

#include "arm_controller.hpp"
#include "arm_config.hpp"
#include "encoder_counter.hpp"

/**
 * @brief 用来储存所有电机，以及实现一些电机相关的helper（使能、处理反转、设零点）
 *
 * 原来通过编译期宏注入的参数（CAN总线、电机ID、校准配置、PD增益）现在全部
 * 通过 ArmConfig 在运行期传入，从而支持一个固件中创建多个 Actuator 实例。
 */
class Actuator {
 public:
  Actuator(rm::hal::CanInterface& lower_can, rm::hal::CanInterface& upper_can, const ArmConfig& config)
      : j1{lower_can, config.dm_settings[0]},
        j2{lower_can, config.dm_settings[1]},
        j3{lower_can, config.dm_settings[2]},
        j4{upper_can, config.dm_settings[3]},
        j5{upper_can, config.dm_settings[4]},
        j6{upper_can, config.dm_settings[5]},
        gripper{upper_can, config.gripper_motor_id} {
    // 从 ArmConfig 初始化校准器
    for (int i = 0; i < 7; ++i) {
      calibrators[i].UpdateConfig(config.calibrators[i]);
    }
    // 从 ArmConfig 拷贝 PD 参数
    for (int i = 0; i < 6; ++i) {
      pd_params_[i] = config.pd_params[i];
    }
  }

  void SetEnable(bool enable, int repeats = 5) {
    const auto dm_cmd = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
    for (int i = 0; i < repeats; ++i) {
      j1.SendInstruction(dm_cmd);
      j2.SendInstruction(dm_cmd);
      j3.SendInstruction(dm_cmd);
      j4.SendInstruction(dm_cmd);
      j5.SendInstruction(dm_cmd);
      j6.SendInstruction(dm_cmd);
      gripper.SetPower(0);
    }
  }

  void ApplyOutput(const ArmController<7>::Output& output, bool apply_position = false) {
    const auto& pd_config = pd_params_;
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
      j4.SetMitCommand(calibrators[3].LogicalToReal(output.positions[3]), 0, 0.f, pd_config[3].kp, pd_config[3].kd);
      j5.SetMitCommand(calibrators[4].LogicalToReal(output.positions[4]), 0, 0.f, pd_config[4].kp, pd_config[4].kd);
      j6.SetMitCommand(calibrators[5].LogicalToReal(output.positions[5]), 0, 0.f, pd_config[5].kp, pd_config[5].kd);
      gripper.SetSpeed(output.velocities[6]);
    } else {
      j1.SetMitCommand(0, 0, 0, 0, 0);
      j2.SetMitCommand(0, 0, 0, 0, 0);
      j3.SetMitCommand(0, 0, 0, 0, 0);
      j4.SetMitCommand(0, 0, 0, 0, 0);
      j5.SetMitCommand(0, 0, 0, 0, 0);
      j6.SetMitCommand(0, 0, 0, 0, 0);
      gripper.SetPower(0);
    }
  }

  void UpdateEncoderCounter() { gripper_encoder.Update(gripper.feedback().position_rad); }

  /**
   * @brief 给达妙电机设零点
   */
  void DmSetZero() {
    SetEnable(true);
    for (int i = 0; i < 10; i++) {
      j4.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
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
    for (int i = 0; i < 7; ++i) {
      if (calibrators[i].reverse()) {
        efforts[i] = -efforts[i];
      }
    }
    return efforts;
  }

  // 校准器保持 public，供 FSM LongJointCalibrating 状态在校准完成后更新零点
  rm::modules::MotorCalibrator calibrators[7]{
      {false, 0.f}, {false, 0.f}, {false, 0.f}, {false, 0.f},
      {false, 0.f}, {false, 0.f}, {false, 0.f}};
  rm::device::DmMotorMit j1, j2, j3, j4, j5, j6;
  rm::device::LkMotorMS<15> gripper;
  EncoderCounter gripper_encoder{2.f * M_PI};

 private:
  detail::PDParams pd_params_[6]{};  ///< 从 ArmConfig 拷贝的 PD 参数
};