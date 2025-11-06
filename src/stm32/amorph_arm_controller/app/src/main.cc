#include <etl/cyclic_value.h>

#include "tim.h"

#include "timer_task.hpp"
#include "globals.hpp"
#include <string>
#include <cstdlib>
#include <cstring>

Globals *globals{nullptr};

// 生成随机字符串，长度在 [min_len, max_len] 字节之间（每次调用重新生成）
static std::string GenerateRandomErrorString(size_t min_len = 100, size_t max_len = 3000) {
  // static bool seeded = false;
  // if (!seeded) {
  //   // 用系统滴答和地址构造种子
  //   unsigned int seed = (unsigned int)HAL_GetTick() ^ (unsigned int)(uintptr_t)&seeded;
  //   srand(seed);
  //   seeded = true;
  // }
  uint32_t random_frame_id_u32;
  HAL_RNG_GenerateRandomNumber(&hrng, &random_frame_id_u32);
  size_t range = (max_len > min_len) ? (max_len - min_len + 1) : 1;
  size_t len = min_len + (random_frame_id_u32 % range);

  static const char *chars =
      "abcdefghijklmnopqrstuvwxyz"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "0123456789"
      "~!@#$%^&*()-_=+[]{};:,.<>?/|\\\"'` ";
  size_t n = strlen(chars);

  std::string s;
  s.reserve(len + 1);
  for (size_t i = 0; i < len; ++i) {
    s.push_back(chars[rand() % n]);
  }
  return s;
}

static void MainLoop() {
  static etl::cyclic_value<int, 0, 300> prescale_3hz;
  static etl::cyclic_value<int, 0, 100> prescale_1hz;

  JointPosition jp;
  jp.positions = {1.0f, 2.0f, 666.0f};

  // 3hz维护RPC连接
  ++prescale_3hz;
  ++prescale_1hz;
  if (prescale_3hz.get() == 0) {
    globals->rpc.KeepConnection();
    globals->rpc.TickTimer();
  }
  if (prescale_1hz.get() == 0) {
    // 每次发送前生成新的随机错误日志（长度 1000~3000 字节）
    static std::string rand_err;
    rand_err = GenerateRandomErrorString(1000, 4000);
    globals->rpc.Error(rand_err.c_str());
    globals->rpc.ReportJointPosition(jp);
  }
}

extern "C" {

[[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.Start();

  globals->buzzer.Beep(2);
  for (;;) {
    __WFI();
  }
}
}