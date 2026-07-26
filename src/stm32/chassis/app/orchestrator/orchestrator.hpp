#pragma once

#include "fsm.hpp"

struct Orchestrator {
  // 状态机
  fsm_orch::OrchestratorFsm fsm;
  fsm_orch::state::Disabled fsm_state_disabled;
  fsm_orch::state::Enabling fsm_state_enabling;
  fsm_orch::state::Normal fsm_state_normal;
  fsm_orch::state::Retracted fsm_state_retracted;
  fsm_orch::state::Fault fsm_state_fault;

 private:
  // 这个数组需与fsm同生命周期，所以放在这里
  etl::ifsm_state *state_list_[6]{
      &fsm_state_disabled,   //
      &fsm_state_enabling,   //
      &fsm_state_normal,     //
      &fsm_state_retracted,  //
      &fsm_state_fault,      //
  };

  // 从串口/DMA ISR 回调中触发的事件不能直接调 fsm.receive()（会导致 reentrancy），
  // 改为缓存，在 Tick() 开头统一消费。
  bool pending_custom_ctrl_{false};
  CustomControllerCommand pending_custom_ctrl_data_{};
  bool pending_host_cmd_{false};
  amorph::schema::ArmCommand pending_host_cmd_data_{};

 public:
  Orchestrator() = default;

  /**
   * @brief 初始化（设备命名、回调注册）
   */
  void Init() {
    auto &shared = SharedResources::GetInstance();

    // 定义遥控器拨杆功能
    shared.rc(rc_ch::SG).OnValueChange([this](const int &old_value, const int &new_value) {
      // 353-up, 1024-mid, 1695-down
      if (old_value == 0) {
        return;  // 第一次触发不响应
      }
      if (new_value > 1300 && new_value < 1700) {  // SD在下
        fsm.receive(fsm_orch::event::SwitchConfigReq{fsm_orch::StateId::kRetracted});
      }
      if (new_value > 0 && new_value < 500) {  // SD在上
        fsm.receive(fsm_orch::event::SwitchConfigReq{fsm_orch::StateId::kNormal});
      }
      if (new_value > 500 && new_value < 1100) {  // SD在中
        fsm.receive(fsm_orch::event::SwitchConfigReq{fsm_orch::StateId::kNormal});
      }
    });

    // 注意，此处涉及到的串口和DMA的中断优先级应该和TIM13保持一致，使两者互斥，防止同时操作状态机
    // 初始化自定义控制器 — ISR 回调中只缓存事件，不直接调 fsm.receive()
    shared.custom_controller.OnRecv([&](const CustomController &cc) {
      pending_custom_ctrl_ = true;
      pending_custom_ctrl_data_ = cc.data();
    });
    // 初始化上位机命令 — 同上，只缓存
    shared.client_broker.OnArmCommandRecv([this](const amorph::schema::ArmCommand &cmd) {
      pending_host_cmd_ = true;
      pending_host_cmd_data_ = cmd;
    });

    fsm.set_states(state_list_, std::size(state_list_));
    fsm.start();
  }

  /**
   * @brief 125 Hz 控制循环节拍（IMU 数据从 SharedResources 内部读取）
   * @note  调用前须已调用 SharedResources::imu.Update()
   */
  void Tick() {
    // 先消费 ISR 回调中缓存的串口事件，避免在 ControlLoop 处理过程中被 ISR 再次触发 fsm.receive() 导致重入
    if (pending_custom_ctrl_) {
      pending_custom_ctrl_ = false;
      fsm.receive(fsm_orch::event::CustomCtrlCommand{pending_custom_ctrl_data_});
    }
    if (pending_host_cmd_) {
      pending_host_cmd_ = false;
      fsm.receive(fsm_orch::event::HostCommand{pending_host_cmd_data_});
    }
    fsm.receive(fsm_orch::event::ControlLoop{});
  }
};