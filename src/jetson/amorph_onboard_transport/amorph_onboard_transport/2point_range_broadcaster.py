"""
CAN FD TwoPointRange Broadcaster.

Subscribes to two sensor_msgs/Range topics and broadcasts their values
as a packed TwoPointRange struct over CAN FD at a fixed rate.
"""

import struct

import can
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Range


# ── struct format matching C++ __attribute__((packed)) TwoPointRange ──
#   bool range_l_valid : 1;   ─┐
#   bool range_r_valid : 1;   ─┘  1 byte (packed bitfields)
#   float range_l;                 4 bytes
#   float range_r;                 4 bytes  →  total 9 bytes
_TPR_STRUCT = struct.Struct('Bff')
assert _TPR_STRUCT.size == 9, f'TwoPointRange struct size mismatch: {_TPR_STRUCT.size} != 9'


class TwoPointRangeBroadcaster(Node):
    """Subscribes to two Range topics and broadcasts them over CAN FD."""

    def __init__(self):
        super().__init__('two_point_range_broadcaster')

        # ── parameter declarations ──
        self.declare_parameter('range_topic_l', '/two_point_range/range_0')
        self.declare_parameter('range_topic_r', '/two_point_range/range_1')
        self.declare_parameter('can_id', 0x210)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('is_extended_id', False)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('range_timeout', 0.5)

        # read parameters
        range_topic_l = (
            self.get_parameter('range_topic_l').get_parameter_value().string_value
        )
        range_topic_r = (
            self.get_parameter('range_topic_r').get_parameter_value().string_value
        )
        self._can_id = (
            self.get_parameter('can_id').get_parameter_value().integer_value
        )
        self._can_interface = (
            self.get_parameter('can_interface').get_parameter_value().string_value
        )
        self._is_extended_id = (
            self.get_parameter('is_extended_id').get_parameter_value().bool_value
        )
        self._publish_rate = (
            self.get_parameter('publish_rate').get_parameter_value().double_value
        )
        self._range_timeout = (
            self.get_parameter('range_timeout').get_parameter_value().double_value
        )

        # ── cached range data ──
        self._range_l = 0.0
        self._range_r = 0.0
        self._range_l_stamp = None
        self._range_r_stamp = None

        # ── subscriptions ──
        self._sub_l = self.create_subscription(
            Range, range_topic_l, self._range_l_callback, 10
        )
        self._sub_r = self.create_subscription(
            Range, range_topic_r, self._range_r_callback, 10
        )

        # ── CAN bus ──
        self._bus = can.interface.Bus(
            channel=self._can_interface,
            interface='socketcan',
            fd=True,
        )

        # ── timer ──
        period_s = 1.0 / max(1.0, self._publish_rate)
        self._timer = self.create_timer(period_s, self._publish_callback)

        self.get_logger().info(
            f'node started: '
            f'range_topic_l={range_topic_l}, '
            f'range_topic_r={range_topic_r}, '
            f'can_id=0x{self._can_id:X}, '
            f'can_interface={self._can_interface}, '
            f'is_extended_id={self._is_extended_id}, '
            f'publish_rate={self._publish_rate} Hz, '
            f'range_timeout={self._range_timeout}s, '
            f'timer period={period_s:.3f}s'
        )

    # ── subscription callbacks ──────────────────────────────────────────

    def _range_l_callback(self, msg: Range):
        self._range_l = float(msg.range)
        self._range_l_stamp = self.get_clock().now()

    def _range_r_callback(self, msg: Range):
        self._range_r = float(msg.range)
        self._range_r_stamp = self.get_clock().now()

    # ── timer callback ──────────────────────────────────────────────────

    def _publish_callback(self):
        """Pack cached range values and broadcast over CAN FD."""
        now = self.get_clock().now()
        timeout_ns = int(self._range_timeout * 1e9)

        # determine validity based on timestamp age
        l_valid = (
            self._range_l_stamp is not None
            and (now.nanoseconds - self._range_l_stamp.nanoseconds) < timeout_ns
        )
        r_valid = (
            self._range_r_stamp is not None
            and (now.nanoseconds - self._range_r_stamp.nanoseconds) < timeout_ns
        )

        # pack two bools into one byte: bit0 = range_l_valid, bit1 = range_r_valid
        flags = (1 if l_valid else 0) | ((1 if r_valid else 0) << 1)

        payload = _TPR_STRUCT.pack(flags, self._range_l, self._range_r)

        msg = can.Message(
            arbitration_id=self._can_id,
            data=payload,
            is_fd=True,
            is_extended_id=self._is_extended_id,
        )

        try:
            self._bus.send(msg)
        except can.CanError as e:
            self.get_logger().warn(
                f'CAN send failed (id=0x{self._can_id:X}): {e}',
                throttle_duration_sec=2.0,
            )


# ── Entry point ────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TwoPointRangeBroadcaster()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
