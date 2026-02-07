/**
 * @brief 操作手可使用自定义控制器通过图传链路向对应的机器人发送数据。
 *        命令码ID：0x0302
 *        字节偏移量 大小 说明
 *        0 30 自定义数据
 *
 *        typedef _packed struct
 *        {
 *          uint8_t data[x];
 *        }custom_robot_data_t;
 *
 *        机器人可通过图传链路向对应的操作手选手端连接的自定义控制器发送数据（RMUL暂不适用）。
 */

#pragma once

#include <cassert>
#include <cstring>
#include <memory>
#include <vector>

#include <librm.hpp>

struct __attribute__((__packed__)) CustomControllerCmd {
  bool ok;
  struct __attribute__((__packed__)) {
    bool a : 1;
    bool b : 1;
    bool c : 1;
    bool d : 1;
    bool e : 1;
    bool reserved_1 : 1;
    bool reserved_2 : 1;
    bool reserved_3 : 1;
  } buttons{};
  float joint_positions[6];
};

static_assert(sizeof(CustomControllerCmd) <= 30, "Payload struct too large");

class PacketBuilder {
 public:
  auto Build(const uint8_t *data, size_t len) -> std::shared_ptr<std::vector<uint8_t>> {
    assert(len < 31);

    std::memcpy(tx_buffer_.data, data, len);
    tx_buffer_.header_crc8 = rm::modules::Crc8(reinterpret_cast<uint8_t *>(&tx_buffer_), 4, rm::modules::CRC8_INIT);
    tx_buffer_.tail_crc16 =
        rm::modules::Crc16(reinterpret_cast<uint8_t *>(&tx_buffer_), sizeof(tx_buffer_) - 2, rm::modules::CRC16_INIT);

    const auto ret = [this] {
      auto v = std::make_shared<std::vector<uint8_t>>(sizeof(tx_buffer_));
      std::memcpy(v->data(), &tx_buffer_, sizeof(tx_buffer_));
      return v;
    }();

    return ret;
  }

 private:
  struct __attribute__((__packed__)) {
    // header
    const uint8_t kSof{0xa5};
    const uint16_t kDataLength{30};
    uint8_t seq{0};
    uint8_t header_crc8;

    // cmdid
    const uint16_t kCmdId{0x0302};

    // payload
    uint8_t data[30]{};

    // tail
    uint16_t tail_crc16;
  } tx_buffer_{};
};
