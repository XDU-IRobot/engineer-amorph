
#include "tim.h"

#include "globals.hpp"

Globals *globals{nullptr};

extern "C" {

[[noreturn]] void AppMain(void) {
  globals = new Globals;

  auto fdcan_transport = std::make_shared<FdCanTransport>(globals->rpc_can, 0x23, 0x33);
  globals->rpc.AttachToConnection(fdcan_transport);

  globals->motor_can1.Begin();
  globals->motor_can2.Begin();
  globals->rpc_can.Begin();
  globals->rc.Begin();

  HAL_TIM_Base_Start_IT(&htim13);

  for (;;) {
    __WFI();
  }
}
}