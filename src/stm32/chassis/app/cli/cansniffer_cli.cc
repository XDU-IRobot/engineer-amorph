// #include "cansniffer_cli.hpp"
//
// #include <chrono>
// #include <cstdlib>
// #include <cstring>
//
// #include <etl/string.h>
// #include <etl/to_string.h>
//
// #include "shared_resources.hpp"
//
// namespace cli {
// namespace program {
//
// namespace {
//
// using Sniffer = decltype(SharedResources::GetInstance().cs);
// using StatsList = Sniffer::StatsList;
//
// StatsList pending_rows;    ///< 尚未打印完的统计结果
// size_t pending_pos{0};     ///< 下一行在pending_rows中的位置
//
// /// 每打印一行需预留的发送缓冲区空间，包含命令回显
// constexpr size_t kRowReserve = 128;
//
// /**
//  * @brief 右对齐追加一个字段
//  */
// void AppendRight(etl::istring &line, const char *text, size_t width) {
//   size_t length = std::strlen(text);
//   for (; length < width; ++length) {
//     line.push_back(' ');
//   }
//   line += text;
// }
//
// void AppendUint(etl::istring &line, rm::u32 value, size_t width) {
//   etl::string<12> text;
//   etl::to_string(value, text);
//   AppendRight(line, text.c_str(), width);
// }
//
// void AppendFloat(etl::istring &line, rm::f32 value, size_t width, uint32_t decimals) {
//   etl::string<20> text;
//   etl::format_spec fmt;
//   fmt.precision(decimals);
//   etl::to_string(value, text, fmt);
//   AppendRight(line, text.c_str(), width);
// }
//
// /**
//  * @brief 追加一个标准帧ID，固定输出为0x000~0x7FF
//  */
// void AppendCanId(etl::istring &line, rm::u16 id, size_t width) {
//   static constexpr char kDigits[] = "0123456789ABCDEF";
//   const char text[] = {'0', 'x', kDigits[(id >> 8) & 0xFu], kDigits[(id >> 4) & 0xFu], kDigits[id & 0xFu], '\0'};
//   AppendRight(line, text, width);
// }
//
// void PrintUsage(EmbeddedCli *cli) {
//   embeddedCliPrint(cli, "CAN sniffer. Usage: cansniffer <list|id|active|dropped>");
//   embeddedCliPrint(cli, "  list          all observed IDs");
//   embeddedCliPrint(cli, "  id <can_id>   stats of one ID, e.g. cansniffer id 0x201");
//   embeddedCliPrint(cli, "  active [ms]   IDs seen in the last <ms>, default 1000");
//   embeddedCliPrint(cli, "  dropped       frames that could not be counted");
// }
//
// void PrintHeader(EmbeddedCli *cli) {
//   etl::string<64> line;
//   AppendRight(line, "ID", 6);
//   AppendRight(line, "DLC", 4);
//   AppendRight(line, "COUNT", 9);
//   AppendRight(line, "AVG(Hz)", 9);
//   AppendRight(line, "DEV(Hz)", 9);
//   embeddedCliPrint(cli, line.c_str());
// }
//
// void PrintRow(EmbeddedCli *cli, const rm::modules::CanIdStats &stats) {
//   etl::string<64> line;
//   AppendCanId(line, stats.id, 6);
//   AppendUint(line, stats.dlc, 4);
//   AppendUint(line, stats.frame_count, 9);
//   AppendFloat(line, stats.average_hz, 9, 1);
//   AppendFloat(line, stats.stddev_hz, 9, 2);
//   embeddedCliPrint(cli, line.c_str());
// }
//
// void PrintDetail(EmbeddedCli *cli, const rm::modules::CanIdStats &stats) {
//   etl::string<48> line;
//
//   line = "ID       ";
//   AppendCanId(line, stats.id, 6);
//   embeddedCliPrint(cli, line.c_str());
//
//   line = "DLC      ";
//   AppendUint(line, stats.dlc, 3);
//   embeddedCliPrint(cli, line.c_str());
//
//   line = "COUNT    ";
//   AppendUint(line, stats.frame_count, 8);
//   embeddedCliPrint(cli, line.c_str());
//
//   line = "AVG      ";
//   AppendFloat(line, stats.average_hz, 8, 1);
//   line += " Hz";
//   embeddedCliPrint(cli, line.c_str());
//
//   line = "STD DEV  ";
//   AppendFloat(line, stats.stddev_hz, 8, 2);
//   line += " Hz";
//   embeddedCliPrint(cli, line.c_str());
// }
//
// void PrintActive(EmbeddedCli *cli, const Sniffer::IdList &ids, rm::u32 timeout_ms) {
//   etl::string<48> header = "Active in last ";
//   AppendUint(header, timeout_ms, 0);
//   header += " ms: ";
//   if (ids.empty()) {
//     header += "none";
//     embeddedCliPrint(cli, header.c_str());
//     return;
//   }
//   AppendUint(header, static_cast<rm::u32>(ids.size()), 0);
//   embeddedCliPrint(cli, header.c_str());
//
//   // 每行最多8个ID，避免单行过长
//   etl::string<64> line;
//   size_t count = 0;
//   for (const rm::u16 id : ids) {
//     AppendCanId(line, id, 6);
//     if (++count % 8 == 0) {
//       embeddedCliPrint(cli, line.c_str());
//       line.clear();
//     }
//   }
//   if (!line.empty()) {
//     embeddedCliPrint(cli, line.c_str());
//   }
// }
//
// }  // namespace
//
// void CanSnifferCommand(EmbeddedCli *cli, char *args, void *ctx) {
//   UNUSED(ctx);
//
//   auto &sniffer = SharedResources::GetInstance().cs;
//
//   const char *subcmd = embeddedCliGetToken(args, 1);
//   if (subcmd == nullptr) {
//     PrintUsage(cli);
//     return;
//   }
//
//   if (std::strcmp(subcmd, "list") == 0) {
//     pending_rows = sniffer.stats();
//     pending_pos = 0;
//     PrintHeader(cli);
//   } else if (std::strcmp(subcmd, "id") == 0) {
//     const char *id_text = embeddedCliGetToken(args, 2);
//     if (id_text == nullptr) {
//       embeddedCliPrint(cli, "Usage: cansniffer id <can_id>");
//       return;
//     }
//     const auto stats = sniffer.stats(static_cast<rm::u16>(std::strtoul(id_text, nullptr, 0)));
//     if (!stats) {
//       embeddedCliPrint(cli, "No frame seen for this ID");
//       return;
//     }
//     PrintDetail(cli, *stats);
//   } else if (std::strcmp(subcmd, "active") == 0) {
//     const char *ms_text = embeddedCliGetToken(args, 2);
//     const rm::u32 timeout_ms = ms_text ? std::strtoul(ms_text, nullptr, 0) : 1000;
//     PrintActive(cli, sniffer.active_ids(std::chrono::milliseconds(timeout_ms)), timeout_ms);
//   } else if (std::strcmp(subcmd, "dropped") == 0) {
//     etl::string<48> line = "Dropped frames: ";
//     AppendUint(line, sniffer.dropped_frames(), 0);
//     embeddedCliPrint(cli, line.c_str());
//   } else {
//     PrintUsage(cli);
//   }
// }
//
// void CanSnifferTick(EmbeddedCli *cli, size_t tx_free) {
//   while (pending_pos < pending_rows.size() && tx_free >= kRowReserve) {
//     PrintRow(cli, pending_rows[pending_pos++]);
//     tx_free -= kRowReserve;
//   }
//   if (pending_pos >= pending_rows.size()) {
//     pending_rows.clear();
//     pending_pos = 0;
//   }
// }
//
// }  // namespace program
// }  // namespace cli
