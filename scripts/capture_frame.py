#!/usr/bin/env python3
"""抓一帧同步的彩色图 + 深度图 + 相机内参，存到目录里供离线调试。

用途：把真实相机数据固化成文件，之后可以在不启动相机、不反复订阅大消息的
情况下调 C++ 节点（配合 amorph_vision_cpp_debug_infer）。

产物（默认 /tmp/eu_frame）：
    color.png        BGR8 彩色图
    depth.npy        float32 深度图，单位毫米
    meta.json        内参 + 原始编码 + 时间戳

注意：这里刻意不使用 cv_bridge —— 系统自带的 cv_bridge 是针对 numpy 1.x 编译的，
在 venv 的 numpy 2.x 下会报警告甚至崩溃。直接按 header 解析 Image.data 更稳。

用法:
    .venv/bin/python scripts/capture_frame.py
    .venv/bin/python scripts/capture_frame.py --outdir /tmp/frame2 --timeout 20
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--outdir", type=Path, default=Path("/tmp/eu_frame"))
    parser.add_argument("--timeout", type=float, default=15.0, help="等待同步帧的秒数")
    parser.add_argument("--rgb-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    return parser.parse_args()


def image_to_array(msg: Image) -> np.ndarray:
    """按 encoding 把 Image.data 解析成 numpy 数组，尊重 step 的填充。"""
    row_bytes = msg.step
    buffer = np.frombuffer(msg.data, dtype=np.uint8)
    if buffer.size < row_bytes * msg.height:
        raise ValueError(f"data 长度不足: {buffer.size} < {row_bytes * msg.height}")
    rows = buffer.reshape(msg.height, row_bytes)

    if msg.encoding in ("rgb8", "bgr8"):
        pixels = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        pixels = pixels.copy()
        return pixels if msg.encoding == "bgr8" else pixels[:, :, ::-1]
    if msg.encoding in ("mono8", "8UC1"):
        return rows[:, : msg.width].copy()
    if msg.encoding in ("16UC1", "mono16"):
        # 每像素 2 字节，按小端解析
        return rows[:, : msg.width * 2].copy().view("<u2").reshape(msg.height, msg.width)
    if msg.encoding == "32FC1":
        return rows[:, : msg.width * 4].copy().view("<f4").reshape(msg.height, msg.width)
    raise ValueError(f"未支持的编码: {msg.encoding}")


class FrameCapture(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("eu_frame_capture")
        self.intrinsics = None
        self.frame = None

        # 相机驱动可能是 BEST_EFFORT，用 SensorData 兼容两种
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

        self.rgb_sub = message_filters.Subscriber(self, Image, args.rgb_topic, qos_profile=qos)
        self.depth_sub = message_filters.Subscriber(self, Image, args.depth_topic, qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.05)
        self.sync.registerCallback(self.on_frame)

        self.create_subscription(CameraInfo, args.camera_info_topic, self.on_camera_info, 10)
        self.get_logger().info("等待同步帧...")

    def on_camera_info(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = {
                "fx": msg.k[0], "fy": msg.k[4], "cx": msg.k[2], "cy": msg.k[5],
                "width": msg.width, "height": msg.height, "frame_id": msg.header.frame_id,
            }

    def on_frame(self, rgb_msg: Image, depth_msg: Image):
        if self.frame is not None:
            return
        try:
            color = image_to_array(rgb_msg)
            raw_depth = image_to_array(depth_msg)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        # 统一成 float32 毫米，与 C++ 节点内部的表示一致
        if depth_msg.encoding in ("16UC1", "mono16"):
            depth = raw_depth.astype(np.float32)
        else:
            depth = raw_depth.astype(np.float32) * 1000.0

        stamp = rgb_msg.header.stamp
        self.frame = {
            "color": color,
            "depth": depth,
            "stamp": stamp.sec + stamp.nanosec * 1e-9,
            "frame_id": rgb_msg.header.frame_id,
            "rgb_encoding": rgb_msg.encoding,
            "depth_encoding": depth_msg.encoding,
        }
        self.get_logger().info(
            f"收到帧: color={color.shape} depth={depth.shape} "
            f"({rgb_msg.encoding}/{depth_msg.encoding})")


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = FrameCapture(args)

    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.frame is not None and node.intrinsics is not None:
                break
    finally:
        frame = node.frame
        intrinsics = node.intrinsics
        node.destroy_node()
        rclpy.shutdown()

    if frame is None:
        print("超时：没有收到同步的彩色/深度帧", file=sys.stderr)
        return 1
    if intrinsics is None:
        print("超时：没有收到 camera_info", file=sys.stderr)
        return 1

    args.outdir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.outdir / "color.png"), frame["color"])
    # 深度同时存 npy（给 Python 用）和 16 位 PNG（给 C++ 用，无需 npy 解析器）
    np.save(str(args.outdir / "depth.npy"), frame["depth"])
    depth_mm = np.clip(np.rint(frame["depth"]), 0, 65535).astype(np.uint16)
    cv2.imwrite(str(args.outdir / "depth.png"), depth_mm)

    valid = frame["depth"] > 0
    meta = {
        **intrinsics,
        "stamp": frame["stamp"],
        "image_frame_id": frame["frame_id"],
        "rgb_encoding": frame["rgb_encoding"],
        "depth_encoding": frame["depth_encoding"],
        "depth_valid_ratio": float(valid.mean()),
        "depth_min_mm": float(frame["depth"][valid].min()) if valid.any() else None,
        "depth_max_mm": float(frame["depth"][valid].max()) if valid.any() else None,
    }
    (args.outdir / "meta.json").write_text(json.dumps(meta, indent=2))

    # C++ 调试工具读这个 key=value 文件，省去 JSON 解析依赖
    (args.outdir / "meta.txt").write_text("".join(
        f"{key}={value}\n" for key, value in {
            "fx": intrinsics["fx"], "fy": intrinsics["fy"],
            "cx": intrinsics["cx"], "cy": intrinsics["cy"],
            "width": intrinsics["width"], "height": intrinsics["height"],
            "frame_id": intrinsics["frame_id"],
            "stamp": frame["stamp"],
        }.items()))

    print(f"已保存到 {args.outdir}")
    print(f"  color.png  {frame['color'].shape}")
    print(f"  depth.png  {depth_mm.shape} uint16 mm, 有效像素 {valid.mean() * 100:.1f}%")
    print(f"  meta.txt   fx={intrinsics['fx']:.1f} fy={intrinsics['fy']:.1f} "
          f"cx={intrinsics['cx']:.1f} cy={intrinsics['cy']:.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
