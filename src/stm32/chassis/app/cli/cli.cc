#include "cli.hpp"

#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"

#include <cstring>

#include <etl/circular_buffer.h>
#include <etl/to_string.h>

#include "bsp/reboot.hpp"
#include "global_objects.hpp"
#include "shared_resources.hpp"

namespace cli {
// 默认配置需约 524 字节，这里分配 1024 字节留足余量（含后续可能新增的 binding）
static CLI_UINT cli_buffer[BYTES_TO_CLI_UINTS(1024)];
EmbeddedCli *cli{nullptr};
etl::circular_buffer<char, 512> tx_buffer;

static void WriteChar(EmbeddedCli *embeddedCli, char c) { tx_buffer.push(c); }

static void PushTxBuffer() {
  static std::atomic<bool> write_done{true};
  if (!write_done.load(std::memory_order_acquire)) {
    return;
  }
  if (tx_buffer.empty()) {
    return;
  }
  write_done.store(false, std::memory_order_release);

  // 从环形缓冲区拷贝到连续数组，供 DMA/IT 发送
  static std::array<char, 512> tx_data;
  size_t len = 0;
  while (!tx_buffer.empty() && len < tx_data.size()) {
    tx_data[len++] = tx_buffer.front();
    tx_buffer.pop();
  }

  SharedResources::GetInstance().no_dtcm->cli_serial.WriteAsync(reinterpret_cast<const uint8_t *>(tx_data.data()), len,
                                                                [] {
                                                                  // in ISR
                                                                  write_done.store(true, std::memory_order_release);
                                                                });
}

/**
 * @brief CLI命令的实现
 */
namespace program {
static void Device(EmbeddedCli *cli, char *args, void *ctx) {
  UNUSED(ctx);

  const auto print_summary_line = [cli](const char *label, const char *summary) {
    etl::string<48> msg;
    msg += "[";
    msg += label;
    msg += "] ";
    msg += (summary[0] == '\0') ? "All devices OK." : summary;
    embeddedCliPrint(cli, msg.c_str());
  };

  const char *subcmd = embeddedCliGetToken(args, 1);
  if (!subcmd) {
    embeddedCliPrint(cli, "Usage: device <summary|status>");
    return;
  }

  if (std::strcmp(subcmd, "summary") == 0) {
    print_summary_line("CHASSIS", chassis->resources.device_manager.GetSummaryString().c_str());
    print_summary_line("GIMBAL", gimbal->resources.device_manager.GetSummaryString().c_str());
    print_summary_line("LEFT_ARM", left_arm->resources.device_manager.GetSummaryString().c_str());
    print_summary_line("RIGHT_ARM", right_arm->resources.device_manager.GetSummaryString().c_str());
  } else if (std::strcmp(subcmd, "status") == 0) {
    const char *dev_name = embeddedCliGetToken(args, 2);
    if (!dev_name) {
      embeddedCliPrint(cli, "Usage: device status <device_name>");
      return;
    }
    // TODO: 按设备名查找并输出详细状态
    etl::string<48> msg;
    msg += dev_name;
    msg += ": not implemented";
    embeddedCliPrint(cli, msg.c_str());
  } else {
    embeddedCliPrint(cli, "Unknown subcommand. Usage: device <summary|status>");
  }
}

static void SetZero(EmbeddedCli *cli, char *args, void *ctx) {
  UNUSED(ctx);

  const char *target_motor_name = embeddedCliGetToken(args, 1);
  if (!target_motor_name) {
    embeddedCliPrint(cli, "invalid argument");
    return;
  }
  if (std::strlen(target_motor_name) != 3) {
    embeddedCliPrint(cli, "invalid argument");
    return;
  }
  if (target_motor_name[1] != 'j') {
    embeddedCliPrint(cli, "invalid argument");
    return;
  }
  if ((target_motor_name[0] == 'l' && left_arm->fsm.get_state_id() != fsm_arm::StateId::kNoForce) ||  //
      (target_motor_name[0] == 'r' && right_arm->fsm.get_state_id() != fsm_arm::StateId::kNoForce)) {
    embeddedCliPrint(cli, "arm not in NoForce mode!");
    return;
  }

  auto &target_arm = target_motor_name[0] == 'l' ? left_arm : right_arm;
  float ecd{0.f};
  switch (target_motor_name[2]) {
    case '1':
      target_arm->resources.actuator.j1.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      ecd = target_arm->resources.actuator.j1.pos();
      break;
    case '2':
      target_arm->resources.actuator.j2.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      ecd = target_arm->resources.actuator.j2.pos();
      break;
    case '3':
      target_arm->resources.actuator.j3.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      ecd = target_arm->resources.actuator.j3.pos();
      break;
    case '4':
      target_arm->resources.actuator.j4.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      ecd = target_arm->resources.actuator.j4.pos();
      break;
    case '5':
      target_arm->resources.actuator.j5.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      ecd = target_arm->resources.actuator.j5.pos();
      break;
    default:
      embeddedCliPrint(cli, "invalid argument");
      return;
  }
  etl::string<48> msg;
  msg += target_motor_name;
  msg += " done, ecd=";
  etl::format_spec fmt;
  fmt.precision(7);
  etl::to_string(ecd, msg, fmt, true);
  embeddedCliPrint(cli, msg.c_str());
}

static void Reboot(EmbeddedCli *cli, char *args, void *ctx) {
  UNUSED(args);
  UNUSED(ctx);
  RebootMCU();
}
}  // namespace program

void Init() {
  static EmbeddedCliConfig *config = embeddedCliDefaultConfig();
  config->cliBuffer = cli_buffer;
  config->cliBufferSize = sizeof(cli_buffer);

  cli = embeddedCliNew(config);
  ETL_ASSERT(cli != nullptr, ETL_ERROR(etl::circular_buffer_empty));  // buffer 不够大会在此处断言失败
  cli->writeChar = WriteChar;

  // 注册命令
  embeddedCliAddBinding(cli,                                                      //
                        {"setzero",                                               //
                         "Set zero position of a motor. Usage: setzero <motor>",  //
                         true,                                                    //
                         nullptr,                                                 //
                         program::SetZero});
  embeddedCliAddBinding(cli,                                                                 //
                        {"device",                                                           //
                         "Device management. Usage: device <summary|status> [device_name]",  //
                         true,                                                               //
                         nullptr,                                                            //
                         program::Device});
  embeddedCliAddBinding(cli,             //
                        {"reboot",       //
                         "Reboot MCU.",  //
                         true,           //
                         nullptr,        //
                         program::Reboot});

  SharedResources::GetInstance().no_dtcm->cli_serial.AttachRxCallback([](etl::span<const uint8_t> data) {
    if (!cli) {
      return;
    }
    for (const uint8_t byte : data) {
      embeddedCliReceiveChar(cli, byte);
    }
  });
}

void Process() {
  embeddedCliProcess(cli);
  PushTxBuffer();
}
}  // namespace cli