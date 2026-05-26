"""
CAN FD EnergyUnitDetection Broadcaster.

Node that polls 8 TF frames (source_frame → energy_unit_0_smoothed ~
energy_unit_7_smoothed) and broadcasts each detection as a 29-byte packed
EnergyUnitDetection frame over CAN FD.  Always sends 8 frames per cycle;
undetected slots have detected=False and zero/identity pose.
"""

import struct
import time

import can
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.clock import ClockType
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformException,
    TransformListener,
)


# ── struct format matching C++ __attribute__((packed)) EnergyUnitDetection ──
#   bool detected;        // 1 byte
#   float p_xyz[3];       // 12 bytes
#   float q_xyzw[4];      // 16 bytes  →  total 29 bytes
_EUD_STRUCT = struct.Struct('=?3f4f')
assert _EUD_STRUCT.size == 29, f'EnergyUnitDetection struct size mismatch: {_EUD_STRUCT.size} != 29'


class CanEuBroadcaster(Node):
    """Polls 8 energy_unit TF frames and broadcasts them over CAN FD."""

    def __init__(self):
        super().__init__('can_eu_broadcaster')

        # -- parameter declarations --
        self.declare_parameter('source_frame', 'camera_link')
        self.declare_parameter('base_can_id', 0x200)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('is_extended_id', False)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('tf_timeout', 0.01)
        self.declare_parameter('detection_timeout', 0.75)

        # read parameters
        self._source_frame = (
            self.get_parameter('source_frame').get_parameter_value().string_value
        )
        self._base_can_id = (
            self.get_parameter('base_can_id').get_parameter_value().integer_value
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
        tf_timeout_s = (
            self.get_parameter('tf_timeout').get_parameter_value().double_value
        )
        self._tf_timeout = Duration(seconds=tf_timeout_s)
        self._detection_timeout = Duration(seconds=
            self.get_parameter('detection_timeout').get_parameter_value().double_value
        )

        # create TF buffer + listener
        self._tf_buffer = Buffer(cache_time=Duration(seconds=1.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # open CAN bus (socketcan, FD mode)
        self._bus = can.interface.Bus(
            channel=self._can_interface,
            interface='socketcan',
            fd=True,
        )

        # create timer
        period_s = 1.0 / max(1.0, self._publish_rate)
        self._timer = self.create_timer(period_s, self._publish_callback)

        self.get_logger().info(
            f'node started: source_frame={self._source_frame}, '
            f'base_can_id=0x{self._base_can_id:X}, '
            f'can_interface={self._can_interface}, '
            f'is_extended_id={self._is_extended_id}, '
            f'publish_rate={self._publish_rate} Hz, '
            f'tf_timeout={tf_timeout_s}s, '
            f'detection_timeout={self._detection_timeout.nanoseconds / 1e9:.3f}s, '
            f'timer period={period_s:.3f}s'
        )

    # ── Timer callback ─────────────────────────────────────────────────────

    def _publish_callback(self):
        """Poll all 8 TF frames and broadcast each over CAN FD."""
        assert self._bus is not None, 'BUS not initialized'
        assert self._tf_buffer is not None, 'TF buffer not initialized'

        self._bus: can.BusABC  # type narrowing hint
        self._tf_buffer: Buffer

        # lookup_time = ROS_TIME zero → TF2 interprets as "latest available"
        lookup_time = Time(clock_type=ClockType.ROS_TIME)
        now = self.get_clock().now()
        source_frame = self._source_frame
        base_id = self._base_can_id
        is_ext = self._is_extended_id
        timeout = self._tf_timeout

        # Try to bring up the CAN bus if it was shut down
        if self._bus is None:
            try:
                self._bus = can.interface.Bus(
                    channel=self._can_interface,
                    interface='socketcan',
                    fd=True,
                )
            except (ValueError, OSError) as e:
                self.get_logger().warn(
                    f'Cannot re-open CAN bus: {e}', throttle_duration_sec=5.0
                )
                return

        for i in range(8):
            child_frame = f'energy_unit_{i}_smoothed'

            try:
                tf = self._tf_buffer.lookup_transform(
                    source_frame,
                    child_frame,
                    lookup_time,
                    timeout,
                )
                # 检查 TF 时间戳是否在 detection_timeout 窗口内
                tf_stamp_ns = tf.header.stamp.sec * int(1e9) + tf.header.stamp.nanosec
                age_ns = now.nanoseconds - tf_stamp_ns
                if age_ns > self._detection_timeout.nanoseconds:
                    raise LookupException(f'TF {child_frame} too old: {age_ns * 1e-9:.3f}s')
                detected = True
                t = tf.transform.translation
                r = tf.transform.rotation
                pos = (t.x, t.y, t.z)
                rot = (r.x, r.y, r.z, r.w)
            except (
                LookupException,
                ConnectivityException,
                ExtrapolationException,
                TransformException,
            ):
                detected = False
                pos = (0.0, 0.0, 0.0)
                rot = (0.0, 0.0, 0.0, 1.0)  # identity quaternion

            payload = _EUD_STRUCT.pack(detected, *pos, *rot)

            msg = can.Message(
                arbitration_id=base_id + i,
                data=payload,
                is_fd=True,
                is_extended_id=is_ext,
            )

            try:
                self._bus.send(msg)
            except can.CanError as e:
                self.get_logger().warn(
                    f'CAN send failed (id=0x{base_id + i:X}): {e}',
                    throttle_duration_sec=2.0,
                )


# ── Entry point ──────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CanEuBroadcaster()
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
