
from crc import Calculator, Configuration

# 初始化CRC计算器（RM协议标准）
# CRC8: 多项式0x31，初始值0x00，MSB-first
_crc8_config = Configuration(
    width=8,
    polynomial=0x31,
    init_value=0x00,
    final_xor_value=0x00,
    reverse_input=False,
    reverse_output=False,
)
_crc8_calculator = Calculator(_crc8_config, optimized=True)

# CRC16: 多项式0xa001，初始值0xffff，LSB-first
_crc16_config = Configuration(
    width=16,
    polynomial=0xa001,
    init_value=0xffff,
    final_xor_value=0x00,
    reverse_input=True,
    reverse_output=False,
)
_crc16_calculator = Calculator(_crc16_config, optimized=True)


class CRC:
    """CRC校验算法"""

    @staticmethod
    def crc8(data, offset, length, init_value, poly=0x31):
        """计算CRC8校验值"""
        return _crc8_calculator.checksum(bytes(data[offset:offset + length])) & 0xFF

    @staticmethod
    def crc16(data, offset, length, init_value, poly=0xa001):
        """计算CRC16校验值"""
        return _crc16_calculator.checksum(bytes(data[offset:offset + length])) & 0xFFFF


class PacketBuilder:
    """
    RM协议数据包构建器
    操作手可使用自定义控制器通过图传链路向对应的机器人发送数据。
    命令码ID：0x0302
    字节偏移量 大小 说明
    0 30 自定义数据
    typedef _packed struct
    {
      uint8_t data[x];
    }custom_robot_data_t;
    机器人可通过图传链路向对应的操作手选手端连接的自定义控制器发送数据（RMUL暂不适用）
    """

    CRC8_INIT = 0x00
    CRC16_INIT = 0xffff

    def __init__(self):
        self.seq = 0

    def build(self, data):
        """构建数据包"""
        if len(data) > 30:
            raise ValueError("Data size exceeds maximum 30 bytes")

        # 初始化30字节的缓冲区
        tx_buffer = bytearray(39)

        # 帧头
        tx_buffer[0] = 0xa5

        # 数据长度（小端）
        tx_buffer[1] = 30
        tx_buffer[2] = 0

        # 序列号
        tx_buffer[3] = self.seq
        self.seq = (self.seq + 1) & 0xFF

        # 计算帧头CRC8（0-3字节）
        header_crc8 = CRC.crc8(tx_buffer, 0, 4, self.CRC8_INIT)
        tx_buffer[4] = header_crc8

        # 命令ID（小端）
        tx_buffer[5] = 0x02
        tx_buffer[6] = 0x03

        # 填充数据（7字节开始）
        for i, byte in enumerate(data):
            tx_buffer[7 + i] = byte

        # 计算尾部CRC16（0-36字节）
        tail_crc16 = CRC.crc16(tx_buffer, 0, 37, self.CRC16_INIT)
        tx_buffer[37] = tail_crc16 & 0xFF
        tx_buffer[38] = (tail_crc16 >> 8) & 0xFF

        return tx_buffer
