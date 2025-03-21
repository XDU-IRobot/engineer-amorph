
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

#ifndef PACKET_BUILDER_HPP
#define PACKET_BUILDER_HPP

#include <cassert>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include <librm.hpp>

class PacketBuilder {
public:
  auto Build(const std::vector<uint8_t> &data) {
    assert(data.size() < 31);

    std::memcpy(tx_buffer_.data, data.data(), data.size());
    tx_buffer_.header_crc8 =
        rm::modules::algorithm::Crc8(reinterpret_cast<uint8_t *>(&tx_buffer_),
                                     4, rm::modules::algorithm::CRC8_INIT);
    tx_buffer_.tail_crc16 = rm::modules::algorithm::Crc16(
        reinterpret_cast<uint8_t *>(&tx_buffer_), sizeof(tx_buffer_) - 2,
        rm::modules::algorithm::CRC16_INIT);

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
    const uint16_t kDataLength{30u};
    uint8_t seq{0};
    uint8_t header_crc8;

    // cmdid
    const uint16_t kCmdId{0x0302};

    // payload
    uint8_t data[30]{0};

    // tail
    uint16_t tail_crc16;
  } tx_buffer_{};
};

#endif