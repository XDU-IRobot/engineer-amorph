#pragma once

#include "schema.hpp"

class PacketBuilder {
 public:
  struct __attribute__((__packed__)) Packet {
    // header
    uint8_t kSof{0xa5};
    uint16_t kDataLength{300};
    uint8_t seq{0};
    uint8_t header_crc8;

    // cmdid
    uint16_t kCmdId{0x310};

    // payload
    // uint8_t data[300]{};
    amorph::schema::RobotFeedBack data;
    uint8_t padding[300 - sizeof(amorph::schema::RobotFeedBack)]{0};

    // tail
    uint16_t tail_crc16;
  };

  static Packet Build(const amorph::schema::RobotFeedBack &rf) {
    static uint8_t seq = 0;

    Packet ret;
    ret.data = rf;
    ret.seq = seq++;
    ret.header_crc8 = rm::modules::Crc8(reinterpret_cast<uint8_t *>(&ret), 4, rm::modules::CRC8_INIT);
    ret.tail_crc16 = rm::modules::Crc16(reinterpret_cast<uint8_t *>(&ret), sizeof(ret) - 2, rm::modules::CRC16_INIT);

    return ret;
  }
};
