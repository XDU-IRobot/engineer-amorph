#pragma once

#include <librm.hpp>

#include "m_pi_f.hpp"

struct Actuator {
  explicit Actuator(rm::hal::CanInterface &can)
      : j1{can, 1},
        j2{can, {.master_id = 0x25, .slave_id = 0x15}},
        j3{can,
           {.master_id = 0x26,
            .slave_id = 0x16,
            .p_max = 12.5f,
            .v_max = 30.f,
            .t_max = 10.f,
            .kp_range = {0, 500},
            .kd_range = {0, 50}}},
        j4{can, 1},
        j5{can, 2},
        j6{can, 3} {
    j1.SetName("j1");
    j2.SetName("j2");
    j3.SetName("j3");
    j4.SetName("j4");
    j5.SetName("j5");
    j6.SetName("j6");
    dev_manager_ << &j1 << &j2 << &j3 << &j4 << &j5 << &j6;
  }

  void SetEnable(bool enable, int repeats = 2) {
    const auto ht_cmd =
        enable ? rm::device::Ht04Motor::Instruction::kEnable : rm::device::Ht04Motor::Instruction::kDisable;
    const auto dm_cmd = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
    const auto lk_cmd = enable ? rm::device::LkMotorMS<15>::kEnable : rm::device::LkMotorMS<15>::kDisable;
    for (int i = 0; i < repeats; ++i) {
      j1.SetCurrent(0);
      j2.SendInstruction(ht_cmd);
      j3.SendInstruction(dm_cmd);
      j4.SendInstruction(lk_cmd);
      j5.SendInstruction(lk_cmd);
      j6.SendInstruction(lk_cmd);
    }
  }

  void UpdateQDD() {
    const auto qd = GetVelocities();
    // d_filter_.update(qd[1], std::chrono::duration_cast<std::chrono::microseconds>(
    //                             std::chrono::steady_clock::now().time_since_epoch())
    //                             .count());
    d_filter_.update(qd[2], HAL_GetTick() * 1000.f);
    qdd_ = d_filter_.slope() * 1e6;
  }

  /**
   * @brief 给电机设零点
   */
  void SetZero() {
    SetEnable(true);
    for (int i = 0; i < 1; i++) {
      // j2.SendInstruction(rm::device::Ht04Motor::Instruction::kSetZeroPosition);
      // j3.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      // j4.SendInstruction(rm::device::LkMotorMS<15>::Instruction::kSetCurrentAsEncoderZero);
      // j5.SendInstruction(rm::device::LkMotorMS<15>::Instruction::kSetCurrentAsEncoderZero);
      // j6.SendInstruction(rm::device::LkMotorMS<15>::Instruction::kSetCurrentAsEncoderZero);
      // HAL_Delay(10);
    }
    SetEnable(false);
  }

  std::array<float, 6> GetPositions() const {
    std::array<float, 6> positions{
        j1.pos_rad(),                //
        j2.pos(),                    //
        j3.pos(),                    //
        j4.feedback().position_rad,  //
        j5.feedback().position_rad,  //
        j6.feedback().position_rad,  //
    };
    // 对每个电机应用反转和零点偏移补偿
    for (size_t i = 0; i < 6; ++i) {
      positions[i] = calibrators[i].RealToLogical(positions[i]);
    }
    // 3个瓴控的电机编码器是0~2pi，需要处理到-pi~pi
    positions[3] = rm::modules::Wrap(positions[3], -M_PI, M_PI);
    positions[4] = rm::modules::Wrap(positions[4], -M_PI, M_PI);
    positions[5] = rm::modules::Wrap(positions[5], -M_PI, M_PI);
    return positions;
  }

  std::array<float, 6> GetRawPositions() const {
    return std::array<float, 6>{
        j1.pos_rad(),                //
        j2.pos(),                    //
        j3.pos(),                    //
        j4.feedback().position_rad,  //
        j5.feedback().position_rad,  //
        j6.feedback().position_rad,  //
    };
  }

  std::array<float, 6> GetVelocities() const {
    std::array<float, 6> velocities{
        j1.rpm() * 2.f * M_PIf / 60.f,  //
        j2.vel(),
        j3.vel(),
        j4.feedback().speed_rad_ps,
        j5.feedback().speed_rad_ps,
        j6.feedback().speed_rad_ps,
    };
    // 对每个电机应用反转
    for (int i = 0; i < 6; ++i) {
      if (calibrators[i].reverse()) {
        velocities[i] = -velocities[i];
      }
    }
    return velocities;
  }

  //---------------------------------------------

  rm::device::GM6020 j1;
  rm::device::Ht04Motor j2;
  rm::device::DmMotorMit j3;
  rm::device::LkMotor<rm::device::LkMotorType::kMS, 15> j4, j5, j6;
  rm::modules::MotorCalibrator calibrators[6]{
#if defined(I_AM_LEFT_CC)
      {false, 4125.f / 8191.f * 2.f * M_PI},                    //
      {false, -(M_PIf - 0.129692078f) + atan2f(84.5f, 415.f)},  //
      {true, -M_PIf / 2.f + atan2f(84.5f, 415.f)},              //
      {false, 0.f},                                             //
      {false, 0.f},                                             //
      {false, 0.f},                                             //
#else defined(I_AM_RIGHT_CC)
      {false, 5690.f / 8191.f * 2.f * M_PI},        //
      {true, M_PIf - atan2f(84.5f, 415.f)},         //
      {false, M_PIf / 2.f - atan2f(84.5f, 415.f)},  //
      {false, 0.f},                                 //
      {true, 0.f},                                  //
      {false, 0.f},                                 //
#endif
  };

  float qdd_;
  rm::modules::DerivativeFilterFloat_Size7 d_filter_;
  rm::device::DeviceManager<6> dev_manager_;
};