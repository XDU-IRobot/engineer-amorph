"""
CAN FD EnergyUnitDetection Broadcaster.

LifecycleNode that polls 8 TF frames (source_frame → energy_unit_0_smoothed ~
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
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
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


class CanEuBroadcaster(LifecycleNode):
    """Polls 8 energy_unit TF frames and broadcasts them over CAN FD."""

    def __init__(self):
        super().__init__('can_eu_broadcaster')

        # -- parameter declarations (read in on_configure) --
        self.declare_parameter('source_frame', 'camera_link')
        self.declare_parameter('base_can_id', 0x200)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('is_extended_id', False)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('tf_timeout', 0.01)

        # -- runtime resources (created in on_configure, destroyed in on_cleanup) --
        self._tf_buffer: Buffer | None = None
        self._tf_listener: TransformListener | None = None
        self._bus: can.BusABC | None = None
        self._timer = None

        # -- cached parameters --
        self._source_frame = ''
        self._base_can_id = 0
        self._can_interface = ''
        self._is_extended_id = False
        self._publish_rate = 20.0
        self._tf_timeout = Duration(seconds=0.01)

        self._active = False

        self.get_logger().info('node created (unconfigured)')

    # ── Lifecycle callbacks ────────────────────────────────────────────────

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('configuring...')

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

        # create TF buffer + listener
        self._tf_buffer = Buffer(cache_time=Duration(seconds=1.0))
        self._tf_listener = TransformListener(self, self._tf_buffer)

        # open CAN bus (socketcan, FD mode)
        try:
            self._bus = can.interface.Bus(
                channel=self._can_interface,
                interface='socketcan',
                fd=True,
            )
        except (ValueError, OSError) as e:
            self.get_logger().error(f'Failed to open CAN bus {self._can_interface}: {e}')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(
            f'configured: source_frame={self._source_frame}, '
            f'base_can_id=0x{self._base_can_id:X}, '
            f'can_interface={self._can_interface}, '
            f'is_extended_id={self._is_extended_id}, '
            f'publish_rate={self._publish_rate} Hz, '
            f'tf_timeout={tf_timeout_s}s'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('activating...')
        period_s = 1.0 / max(1.0, self._publish_rate)
        self._timer = self.create_timer(period_s, self._publish_callback)
        self._active = True
        self.get_logger().info(f'activated, timer period={period_s:.3f}s')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('deactivating...')
        self._active = False
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except can.CanError:
                pass
            # Bus will be re-created in on_configure
        self.get_logger().info('deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('cleaning up...')

        self._active = False

        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

        if self._bus is not None:
            try:
                self._bus.shutdown()
            except can.CanError:
                pass
            self._bus = None

        if self._tf_listener is not None:
            # Detach the listener from the node (implicitly via destruction)
            self._tf_listener = None

        self._tf_buffer = None

        self.get_logger().info('cleanup complete')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('shutting down...')
        return self.on_cleanup(state)

    # ── Timer callback ─────────────────────────────────────────────────────

    def _publish_callback(self):
        """Poll all 8 TF frames and broadcast each over CAN FD."""
        assert self._bus is not None, 'BUS not initialized'
        assert self._tf_buffer is not None, 'TF buffer not initialized'

        self._bus: can.BusABC  # type narrowing hint
        self._tf_buffer: Buffer

        now = Time()
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
                    now,
                    timeout,
                )
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
