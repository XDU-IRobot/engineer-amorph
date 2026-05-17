#!/usr/bin/env python3
"""
UDP H.265 Video Transport Receiver Node.

Receives fragmented H.265 video stream from RoboMaster custom client's
UDP port 3334, reassembles frames, decodes them via GStreamer, and
publishes as ROS 2 Image/CameraInfo topics for RViz2 Camera display.

Protocol (per RoboMaster 2026 communication spec, §1.4.2 / custom client):
    Each UDP packet has an 8-byte custom header followed by H.265 data:
    - frame_num:   uint16 (big-endian) — incrementing frame counter
    - frag_seq:    uint16 (big-endian) — fragment sequence within this frame
    - total_size:  uint32 (big-endian) — total byte size of the complete frame
    - payload:     remaining bytes are H.265 bitstream data
"""

import socket
import struct
import threading
import time
from collections import defaultdict
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo

# ---------------------------------------------------------------------------
#  fragment parser — UDP → reassembled H.265 frames
# ---------------------------------------------------------------------------

HEADER_FMT = '>HHI'          # frame_num, frag_seq, total_size
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 8 bytes
MAX_FRAME_AGE_SEC = 5.0      # discard incomplete frames older than this


class FragmentParser:
    """
    Binds a UDP socket on *port*, reads the custom 8-byte header, and
    reassembles complete H.265 frames.

    Thread-safe: designed to be called from a dedicated I/O thread.
    """

    def __init__(self, port: int = 3334):
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Per-frame state: {frame_num: {total_size, received_bytes, fragments: {seq: bytes}, last_seen}}
        self._pending: dict[int, dict] = defaultdict(
            lambda: {'total_size': 0, 'received_bytes': 0, 'fragments': {}, 'last_seen': 0.0}
        )
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # -- public API -----------------------------------------------------------

    def start(self):
        """Bind socket and start reader thread."""
        self._sock.bind(('0.0.0.0', self._port))
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True, name='udp-reader')
        self._thread.start()

    def stop(self):
        """Stop reader and close socket."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            self._sock.close()
        except OSError:
            pass

    def get_next_frame(self) -> Optional[bytes]:
        """
        Non-blocking: return a complete H.265 frame if one is available,
        otherwise return None.  Call regularly from the ROS timer/game-loop.
        Also performs stale-frame garbage collection.
        """
        now = time.monotonic()
        completed = []          # list of frame_nums ready to return
        stale = []              # frame_nums to discard

        for frame_num, state in list(self._pending.items()):
            if state['total_size'] > 0 and state['received_bytes'] >= state['total_size']:
                completed.append(frame_num)
            elif now - state['last_seen'] > MAX_FRAME_AGE_SEC:
                stale.append(frame_num)

        # clean stale frames
        for fnum in stale:
            del self._pending[fnum]

        if not completed:
            return None

        # return the oldest completed frame
        completed.sort()
        frame_num = completed[0]
        state = self._pending.pop(frame_num)
        # sort fragments by seq and concatenate
        frags = [state['fragments'][seq] for seq in sorted(state['fragments'].keys())]
        return b''.join(frags)

    # -- internal -------------------------------------------------------------

    def _reader_loop(self):
        """Blocking UDP read loop (runs in daemon thread)."""
        while self._running:
            try:
                data, _ = self._sock.recvfrom(65536)
                self._process_packet(data)
            except (socket.timeout, BlockingIOError):
                continue
            except OSError:
                if self._running:
                    raise
                break

    def _process_packet(self, data: bytes):
        if len(data) < HEADER_SIZE:
            return  # malformed

        frame_num, frag_seq, total_size = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
        payload = data[HEADER_SIZE:]
        state = self._pending[frame_num]
        state['total_size'] = total_size
        state['fragments'][frag_seq] = payload
        state['received_bytes'] += len(payload)
        state['last_seen'] = time.monotonic()


# ---------------------------------------------------------------------------
#  GStreamer H.265 → BGR decoder
# ---------------------------------------------------------------------------

GST_PIPELINE = (
    'appsrc name=src is-live=true block=true format=time '
    '! h265parse '
    '! avdec_h265 '
    '! videoconvert '
    '! video/x-raw,format=BGR '
    '! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false'
)

# Defer imports so they don't break at import-time if GStreamer is not installed.
_gi_imported = False
_Gst = None
_GObject = None

def _ensure_gi():
    global _Gst, _GObject, _gi_imported
    if _gi_imported:
        return
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst, GObject  # noqa: E402
    Gst.init(None)
    _Gst = Gst
    _GObject = GObject
    _gi_imported = True


class VideoDecoder:
    """
    Wraps a GStreamer pipeline that decodes H.265 byte-stream into BGR frames.

    Usage::

        decoder = VideoDecoder(on_frame=<callback>)
        decoder.start()
        decoder.push_frame(h265_bytes)
        ...
        decoder.stop()
    """

    def __init__(self, on_frame):
        """
        Args:
            on_frame: callable(bgr_ndarray) invoked in a GStreamer thread
                      whenever a decoded frame is available.
        """
        _ensure_gi()
        self._on_frame = on_frame
        self._pipeline = _Gst.parse_launch(GST_PIPELINE)
        self._appsrc = self._pipeline.get_by_name('src')
        self._appsink = self._pipeline.get_by_name('sink')
        self._appsink.connect('new-sample', self._on_new_sample)

    def start(self):
        self._pipeline.set_state(_Gst.State.PLAYING)

    def stop(self):
        self._pipeline.set_state(_Gst.State.NULL)

    def push_frame(self, data: bytes):
        """
        Push a complete H.265 frame into the pipeline.  Must be called
        from an arbitrary thread (GStreamer handles thread-safety).
        """
        buf = _Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        ret = self._appsrc.emit('push-buffer', buf)
        if ret != _Gst.FlowReturn.OK:
            # Could log warning here in a real node
            pass

    def _on_new_sample(self, sink) -> _Gst.FlowReturn:
        sample = sink.emit('pull-sample')
        if sample is None:
            return _Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()
        struct_info = caps.get_structure(0)
        width = struct_info.get_int('width')[1]
        height = struct_info.get_int('height')[1]

        _success, map_info = buf.map(_Gst.MapFlags.READ)
        if not _success:
            return _Gst.FlowReturn.ERROR

        try:
            # Convert raw BGR bytes → numpy array
            import numpy as np
            frame = np.ndarray(
                shape=(height, width, 3),
                dtype=np.uint8,
                buffer=map_info.data,
            ).copy()  # copy because map_info will be unmapped
        finally:
            buf.unmap(map_info)

        self._on_frame(frame)
        return _Gst.FlowReturn.OK


# ---------------------------------------------------------------------------
#  ROS 2 node
# ---------------------------------------------------------------------------

class UdpVideoReceiverNode(Node):
    """ROS 2 node that bridges UDP H.265 stream → Image + CameraInfo topics."""

    def __init__(self):
        super().__init__('udp_video_receiver')

        # -- parameters -------------------------------------------------------
        self.declare_parameter('udp_port', 3334)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('poll_period_sec', 0.01)   # 100 Hz poll

        udp_port = self.get_parameter('udp_port').value
        self._frame_id = self.get_parameter('frame_id').value
        poll_period = self.get_parameter('poll_period_sec').value

        # -- publishers -------------------------------------------------------
        # Best-effort for video (low latency), transient-local for CameraInfo
        video_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        info_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._img_pub = self.create_publisher(Image, 'image_raw', video_qos)
        self._cinfo_pub = self.create_publisher(CameraInfo, 'camera_info', info_qos)

        # -- CvBridge ---------------------------------------------------------
        from cv_bridge import CvBridge
        self._bridge = CvBridge()

        # -- fragment parser + decoder -----------------------------------------
        self._parser = FragmentParser(port=udp_port)
        self._parser.start()

        self._decoder = VideoDecoder(on_frame=self._on_decoded_frame)
        self._decoder.start()

        # -- polling timer (feeds parser → decoder) ---------------------------
        self._timer = self.create_timer(poll_period, self._poll)

        self.get_logger().info(
            f'Listening on UDP :{udp_port}, '
            f'publishing to image_raw + camera_info'
        )

    # -- cleanup ---------------------------------------------------------------

    def destroy_node(self):
        self._decoder.stop()
        self._parser.stop()
        super().destroy_node()

    # -- periodic poll ---------------------------------------------------------

    def _poll(self):
        """Pull assembled frames from the parser and push them into the decoder."""
        frame = self._parser.get_next_frame()
        if frame is not None:
            self._decoder.push_frame(frame)

    # -- frame callback (GStreamer thread) ------------------------------------

    def _on_decoded_frame(self, bgr_frame):
        """
        Called from GStreamer thread whenever a decoded BGR frame is ready.
        Publishes Image + static CameraInfo.
        """
        import numpy as np

        stamp = self.get_clock().now().to_msg()
        height, width = bgr_frame.shape[:2]

        # Convert BGR → sensor_msgs/Image
        img_msg = self._bridge.cv2_to_imgmsg(bgr_frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self._frame_id
        self._img_pub.publish(img_msg)

        # Publish CameraInfo (static, re-sent on each frame for simplicity)
        cinfo = CameraInfo()
        cinfo.header.stamp = stamp
        cinfo.header.frame_id = self._frame_id
        cinfo.width = width
        cinfo.height = height
        # Default intrinsics: unit focal length, centered principal point.
        # Replace with real calibration if available.
        cinfo.k[0] = float(width)
        cinfo.k[2] = float(width) / 2.0
        cinfo.k[4] = float(width)
        cinfo.k[5] = float(height) / 2.0
        cinfo.k[8] = 1.0
        # No distortion
        cinfo.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        cinfo.p = cinfo.k
        cinfo.p[3] = 0.0
        cinfo.p[7] = 0.0
        cinfo.binning_x = 1
        cinfo.binning_y = 1
        cinfo.roi.x_offset = 0
        cinfo.roi.y_offset = 0
        cinfo.roi.width = width
        cinfo.roi.height = height
        cinfo.roi.do_rectify = False
        self._cinfo_pub.publish(cinfo)


def main(args=None):
    rclpy.init(args=args)
    node = UdpVideoReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
