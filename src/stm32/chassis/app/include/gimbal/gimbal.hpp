#pragma once

#include "global_config.hpp"
#include "resources.hpp"
#include "fsm.hpp"

struct Gimbal {
  GimbalResources resources;

  // 状态机
  fsm_gimbal::Gimbal fsm{resources};
  fsm_gimbal::state::NoForce fsm_state_no_force;
  fsm_gimbal::state::Up fsm_state_up;
  fsm_gimbal::state::Mid fsm_state_mid;
  fsm_gimbal::state::Down fsm_state_down;
  fsm_gimbal::state::ForceDown fsm_state_force_down;

 private:
  // 这个数组需与fsm同生命周期，所以放在这里
  etl::ifsm_state *state_list_[5]{
      &fsm_state_no_force,  //
      &fsm_state_up,        //
      &fsm_state_mid,       //
      &fsm_state_down,      //
      &fsm_state_force_down,
  };

 public:
  explicit Gimbal(const GimbalConfig &config) : resources{config} {}

  /**
   * @brief 初始化（设备命名、回调注册）
   */
  void Init() {
    auto &shared = SharedResources::GetInstance();

    // 定义遥控器拨杆功能，353-up, 1024-mid, 1695-down
    shared.rc(rc_ch::SA).OnValueChange([&](const int &old_value, const int &new_value) {
      if (old_value == 0) {  // 第一次触发不响应
        return;
      }
      if (new_value > 1300 && new_value < 1700) {  // SA在下，云台无力
        fsm.receive(fsm_gimbal::event::ForceModeSwitch{fsm_gimbal::StateId::kNoForce});
      }
    });
    shared.rc(rc_ch::SE).OnValueChange([&](const int &old_value, const int &new_value) {
      const int16_t sa_value = shared.rc.channel(rc_ch::SA);
      if (old_value == 0 ||                     // 第一次触发不响应
          sa_value < 500 || sa_value > 1100) {  // SA不在中间（云台不在遥控器调试模式），不响应
        return;
      }

      // 调试模式（SA在中间）下可以用SE控制云台模式
      if (new_value > 1300 && new_value < 1700) {  // SE在下
        fsm.receive(fsm_gimbal::event::ForceModeSwitch{fsm_gimbal::StateId::kDown});
      }
      if (new_value > 0 && new_value < 500) {  // SE在上
        fsm.receive(fsm_gimbal::event::ForceModeSwitch{fsm_gimbal::StateId::kUp});
      }
      if (new_value > 500 && new_value < 1100) {  // SE在中
        fsm.receive(fsm_gimbal::event::ForceModeSwitch{fsm_gimbal::StateId::kMid});
      }
    });

    // 定义键盘键位
    using RC = rm::device::RefereeProtocol<GlobalConfig::kRefereeRev>::RemoteControl;
    shared.kh
        .OnKeyDown(RC::kF,
                   [&](const rm::modules::KeyEvent &e) {  // 杆升
                     fsm.receive(fsm_gimbal::event::ForceModeSwitch{fsm_gimbal::StateId::kUp});
                   })
        .OnKeyDown(RC::kC, [&](const rm::modules::KeyEvent &e) {  // 杆降
          fsm.receive(fsm_gimbal::event::ForceModeSwitch{fsm_gimbal::StateId::kDown});
        });

    fsm.set_states(state_list_, std::size(state_list_));
    fsm.start();
  }

  /**
   * @brief 250 Hz 控制循环节拍
   */
  void Tick() {
    resources.device_manager.Update();
    fsm.receive(fsm_gimbal::event::ControlLoop{});
    // 底盘的Tick已经调用了SendCommand，这里就不用了
    // rm::device::DjiMotorBase::SendCommand(SharedResources::GetInstance().mcp_can);
  }
};