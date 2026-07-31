"""Foxglove WebSocket 可视化: 发布 CameraFrame 和检测框."""

from __future__ import annotations

import colorsys
import logging
import struct

import cv2
import numpy as np
from foxglove import start_server  # type: ignore
from foxglove.channels import (  # type: ignore
    CameraCalibrationChannel,
    CompressedImageChannel,
    FrameTransformChannel,
    ImageAnnotationsChannel,
    PointCloudChannel,
    LogChannel
)
from foxglove.messages import (  # type: ignore
    CameraCalibration,
    Color,
    CompressedImage,
    FrameTransform,
    ImageAnnotations,
    Log,
    LogLevel,
    PackedElementField,
    PackedElementFieldNumericType,
    Point2,
    PointCloud,
    PointsAnnotation,
    PointsAnnotationType,
    Pose,
    Quaternion,
    TextAnnotation,
    Timestamp,
    Vector3,
)

from .cam import CameraFrame

logger = logging.getLogger(__name__)


class FoxgloveServer:
    def __init__(self, host: str = "0.0.0.0", port: int = 8765) -> None:
        self._host = host
        self._port = port
        self._server = None
        self._rgb_chan = None
        self._depth_chan = None
        self._calib_chan = None
        self._annot_chan = None
        self._pcl_chans: dict[str, PointCloudChannel] = {}
        self._tf_chan = None
        self._log_chan = None

    def start(self) -> None:
        if self._server is not None:
            return
        self._server = start_server(host=self._host, port=self._port)
        self._rgb_chan = CompressedImageChannel(topic="/camera/color/compressed")
        self._depth_chan = CompressedImageChannel(topic="/camera/depth/compressed")
        self._calib_chan = CameraCalibrationChannel(topic="/camera/calibration")
        self._annot_chan = ImageAnnotationsChannel(topic="/camera/annotations")
        self._tf_chan = FrameTransformChannel(topic="/tf")
        self._log_chan = LogChannel(topic="/log")
        logger.info("Foxglove server ws://%s:%d", self._host, self._port)

    def stop(self) -> None:
        for chan in (self._rgb_chan, self._depth_chan, self._calib_chan, self._annot_chan, self._tf_chan, self._log_chan):
            if chan is not None:
                chan.close()
        for chan in self._pcl_chans.values():
            chan.close()
        if self._server is not None:
            self._server.stop()
        self._rgb_chan = self._depth_chan = self._calib_chan = self._annot_chan = self._tf_chan = self._log_chan = None
        self._pcl_chans.clear()
        self._server = None

    def publish_frame(self, frame: CameraFrame, *, frame_id: str = "camera_optical_frame") -> None:
        t = Timestamp.now()

        # world (Z-up) → camera_optical_frame (Z-forward)
        if self._tf_chan is not None:
            self._tf_chan.log(FrameTransform(
                timestamp=t,
                parent_frame_id="world",
                child_frame_id="camera_optical_frame",
                translation=Vector3(x=0, y=0, z=0),
                rotation=Quaternion(x=-0.5, y=0.5, z=-0.5, w=0.5),
            ))

        _, rgb_jpeg = cv2.imencode(".jpg", frame.color, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        self._rgb_chan.log(CompressedImage(
            timestamp=t, frame_id=frame_id, data=rgb_jpeg.tobytes(), format="jpeg",
        ))

        dh, dw = frame.depth.shape
        depth_display = _normalize_depth(frame.depth)
        _, depth_jpeg = cv2.imencode(".jpg", depth_display, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        self._depth_chan.log(CompressedImage(
            timestamp=t, frame_id=frame_id, data=depth_jpeg.tobytes(), format="jpeg",
        ))

        K = frame.K
        R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        P = [K[0], K[1], K[2], 0.0, K[3], K[4], K[5], 0.0, K[6], K[7], K[8], 0.0]
        self._calib_chan.log(CameraCalibration(
            timestamp=t, frame_id=frame_id,
            width=dw, height=dh,
            distortion_model="plumb_bob",
            D=[], K=list(K), R=R, P=P,
        ))

    def publish_yolo_result(self, results) -> None:
        """发布 YOLO 推理结果: bbox + 标签 + mask 轮廓(如有)."""
        if not results:
            return

        points_anns = []
        text_anns = []

        for r in results:
            if r.boxes is None:
                continue

            has_masks = r.masks is not None and hasattr(r.masks, 'xy') and len(r.masks.xy) > 0

            for i in range(len(r.boxes)):
                cls_id = int(r.boxes.cls[i].item())
                conf = float(r.boxes.conf[i].item())
                x1, y1, x2, y2 = r.boxes.xyxy[i].tolist()
                label = r.names[cls_id]

                h = hash(label) % 360 / 360.0
                r_c, g_c, b_c = colorsys.hsv_to_rgb(h, 0.85, 0.95)
                color = Color(r=r_c, g=g_c, b=b_c, a=0.8)
                fill_color = Color(r=r_c, g=g_c, b=b_c, a=0.2)

                # bbox
                pts = [
                    Point2(x=x1, y=y1), Point2(x=x2, y=y1),
                    Point2(x=x2, y=y2), Point2(x=x1, y=y2),
                ]
                points_anns.append(PointsAnnotation(
                    type=PointsAnnotationType.LineLoop,
                    points=pts, outline_color=color, thickness=2,
                ))

                # 标签
                text_y = max(0, y1 - 2)
                text_anns.append(TextAnnotation(
                    position=Point2(x=x1, y=text_y),
                    text=f"{label} {conf:.2f}",
                    font_size=14,
                    text_color=Color(r=1, g=1, b=1, a=1),
                    background_color=color,
                ))

                # mask 轮廓
                if has_masks and i < len(r.masks.xy):
                    poly = r.masks.xy[i]
                    if len(poly) >= 3:
                        mask_pts = [Point2(x=float(p[0]), y=float(p[1])) for p in poly]
                        points_anns.append(PointsAnnotation(
                            type=PointsAnnotationType.LineLoop,
                            points=mask_pts,
                            outline_color=color,
                            fill_color=fill_color,
                            thickness=1,
                        ))

        self._annot_chan.log(ImageAnnotations(
            points=points_anns, texts=text_anns,
        ))

    def _get_or_create_pcl_chan(self, pcl_name: str) -> PointCloudChannel:
        if pcl_name not in self._pcl_chans:
            self._pcl_chans[pcl_name] = PointCloudChannel(topic=f"/pcl/{pcl_name}")
        return self._pcl_chans[pcl_name]

    def publish_point_cloud(self, frame: CameraFrame, *, frame_id: str = "camera_optical_frame", pcl_name: str = "camera") -> None:
        """发布彩色点云."""
        if frame.points is None:
            return
        pcl_chan = self._get_or_create_pcl_chan(pcl_name)

        pts = frame.points
        buf = bytearray(len(pts) * 16)
        for i, (x, y, z, r, g, b) in enumerate(pts):
            rgba = (int(b) << 24) | (int(g) << 16) | (int(r) << 8) | 0xFF
            struct.pack_into("<fffI", buf, i * 16, x * 0.001, y * 0.001, z * 0.001, rgba)

        f32 = PackedElementFieldNumericType.Float32
        u32 = PackedElementFieldNumericType.Uint32
        pcl_chan.log(PointCloud(
            timestamp=Timestamp.now(),
            frame_id=frame_id,
            pose=Pose(position=Vector3(x=0, y=0, z=0),
                       orientation=Quaternion(x=0, y=0, z=0, w=1)),
            point_stride=16,
            fields=[
                PackedElementField(name="x", offset=0, type=f32),
                PackedElementField(name="y", offset=4, type=f32),
                PackedElementField(name="z", offset=8, type=f32),
                PackedElementField(name="rgba", offset=12, type=u32),
            ],
            data=bytes(buf),
        ))

    def log(self, message: str, level: LogLevel = LogLevel.Info, name: str = "") -> None:
        """发布日志消息到 /log topic."""
        if self._log_chan is None:
            return
        self._log_chan.log(Log(
            timestamp=Timestamp.now(),
            level=level,
            message=message,
            name=name,
        ))

    def publish_tf(
        self,
        parent_frame_id: str,
        child_frame_id: str,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
    ) -> None:
        """发布坐标系变换.

        Parameters
        ----------
        translation : (x, y, z) 米
        rotation : (x, y, z, w) 四元数
        """
        if self._tf_chan is None:
            return
        self._tf_chan.log(FrameTransform(
            timestamp=Timestamp.now(),
            parent_frame_id=parent_frame_id,
            child_frame_id=child_frame_id,
            translation=Vector3(x=translation[0], y=translation[1], z=translation[2]),
            rotation=Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3]),
        ))

    def publish_point_cloud_data(
        self, points: np.ndarray, *, frame_id: str = "camera_optical_frame", pcl_name: str = "model"
    ) -> None:
        """发布通用 N×6 点云.

        Parameters
        ----------
        points : np.ndarray, shape (N, 6), float32
            每行 (x, y, z, r, g, b), xyz 单位米, rgb 0-255.
        """
        pcl_chan = self._get_or_create_pcl_chan(pcl_name)
        pts = np.asarray(points, dtype=np.float32)
        buf = bytearray(len(pts) * 16)
        for i, (x, y, z, rr, gg, bb) in enumerate(pts):
            rgba = (int(bb) << 24) | (int(gg) << 16) | (int(rr) << 8) | 0xFF
            struct.pack_into("<fffI", buf, i * 16, x, y, z, rgba)

        f32 = PackedElementFieldNumericType.Float32
        u32 = PackedElementFieldNumericType.Uint32
        pcl_chan.log(PointCloud(
            timestamp=Timestamp.now(),
            frame_id=frame_id,
            pose=Pose(position=Vector3(x=0, y=0, z=0),
                       orientation=Quaternion(x=0, y=0, z=0, w=1)),
            point_stride=16,
            fields=[
                PackedElementField(name="x", offset=0, type=f32),
                PackedElementField(name="y", offset=4, type=f32),
                PackedElementField(name="z", offset=8, type=f32),
                PackedElementField(name="rgba", offset=12, type=u32),
            ],
            data=bytes(buf),
        ))


def _normalize_depth(depth: np.ndarray) -> np.ndarray:
    """float32 mm depth → 8-bit 灰度图."""
    valid = depth[depth > 0]
    if len(valid) == 0 or valid.max() == valid.min():
        return np.zeros(depth.shape, dtype=np.uint8)
    normalized = (depth - valid.min()) / (valid.max() - valid.min()) * 255.0
    return np.clip(normalized, 0, 255).astype(np.uint8)
