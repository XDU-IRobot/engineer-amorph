// #pragma once
//
// #include <cstddef>
//
// #include "embedded_cli.h"
//
// namespace cli {
// namespace program {
//
// /**
//  * @brief cansniffer命令，查询CAN总线各ID的活跃情况和出现频率
//  * @note  不带参数时打印用法
//  */
// void CanSnifferCommand(EmbeddedCli *cli, char *args, void *ctx);
//
// /**
//  * @brief 继续输出上一条命令尚未打印完的统计行
//  * @param cli      CLI对象
//  * @param tx_free  CLI发送缓冲区的剩余空间，单位字节
//  * @note  一次能打印的行数受串口发送缓冲区限制，剩余的表格行在主循环里逐次补完
//  */
// void CanSnifferTick(EmbeddedCli *cli, size_t tx_free);
//
// }  // namespace program
// }  // namespace cli
