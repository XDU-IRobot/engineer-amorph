"""Camera interface for Orbbec Gemini 336L."""

from __future__ import annotations

import time
from dataclasses import dataclass

import cv2
import numpy as np
from pyorbbecsdk import (
    AlignFilter, Config, Context, DeviceList, FormatConvertFilter,
    OBConvertFormat, OBFormat, OBFrameAggregateOutputMode, OBLogLevel,
    OBPropertyID, OBSensorType, OBStreamType, Pipeline,
    PointCloudFilter, VideoFrame,
)


@dataclass
class CameraFrame:
    color: np.ndarray       # BGR (H,W,3) uint8
    depth: np.ndarray       # mm (H,W) float32
    K: list[float]          # depth intrinsics [fx,0,cx, 0,fy,cy, 0,0,1]
    points: np.ndarray | None = None  # Nx6 (x_mm,y_mm,z_mm,r,g,b) float32 或 None


def _get_convert_format(frame: VideoFrame):
    fmt = frame.get_format()
    if fmt == OBFormat.I420: return OBConvertFormat.I420_TO_RGB888
    if fmt == OBFormat.MJPG: return OBConvertFormat.MJPG_TO_RGB888
    if fmt == OBFormat.YUYV: return OBConvertFormat.YUYV_TO_RGB888
    if fmt == OBFormat.NV21: return OBConvertFormat.NV21_TO_RGB888
    if fmt == OBFormat.NV12: return OBConvertFormat.NV12_TO_RGB888
    if fmt == OBFormat.UYVY: return OBConvertFormat.UYVY_TO_RGB888
    return None


def _to_rgb_frame(frame: VideoFrame) -> VideoFrame | None:
    if frame.get_format() == OBFormat.RGB:
        return frame
    cf = _get_convert_format(frame)
    if cf is None:
        return None
    f = FormatConvertFilter()
    f.set_format_convert_format(cf)
    return f.process(frame)


class Camera:
    WIDTH, HEIGHT, FPS = 1280, 800, 10

    def __init__(self):
        self._pipeline: Pipeline | None = None
        self._ctx: Context | None = None
        self._align_filter: AlignFilter | None = None
        self._point_cloud_filter: PointCloudFilter | None = None
        self._disconnected = False

    def _on_device_changed(self, removed: DeviceList, added: DeviceList):
        if removed.get_count() > 0:
            self._disconnected = True

    def open(self) -> None:
        self._ctx = Context()
        self._ctx.set_logger_level(OBLogLevel.WARNING)

        device_list = self._ctx.query_devices()
        if device_list.get_count() == 0:
            raise RuntimeError("No Orbbec device found")

        self._pipeline = Pipeline()
        config = self._set_config()
        self._align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
        self._point_cloud_filter = PointCloudFilter()
        self._point_cloud_filter.set_create_point_format(OBFormat.RGB_POINT)

        device = self._pipeline.get_device()
        device.load_preset("High Accuracy")
        device.set_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, True)
        device.set_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, True)
        device.set_bool_property(OBPropertyID.OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, True)

        self._ctx.set_device_changed_callback(self._on_device_changed)
        self._pipeline.start(config)

    def _set_config(self) -> Config:
        config = Config()
        color_profiles = self._pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = color_profiles.get_video_stream_profile(
            self.WIDTH, self.HEIGHT, OBFormat.RGB, self.FPS)
        config.enable_stream(color_profile)

        depth_profiles = self._pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        depth_profile = depth_profiles.get_video_stream_profile(
            self.WIDTH, self.HEIGHT, OBFormat.UNKNOWN_FORMAT, self.FPS)
        config.enable_stream(depth_profile)

        config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
        return config

    def close(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
        self._ctx = None

    def wait_for_frame(self, timeout_ms: int = 1000) -> CameraFrame | None:
        if self._pipeline is None:
            raise RuntimeError("Camera not opened")

        deadline = time.monotonic() + timeout_ms / 1000.0
        while True:
            if self._disconnected:
                raise RuntimeError("Camera disconnected")

            remaining = max(0, int((deadline - time.monotonic()) * 1000))
            fs = self._pipeline.wait_for_frames(remaining)
            if fs is None:
                return None

            aligned = self._align_filter.process(fs)
            if aligned is None:
                continue

            cf = aligned.get_color_frame()
            df = aligned.get_depth_frame()
            if cf is None or df is None:
                continue

            rgb = _to_rgb_frame(cf)
            if rgb is None:
                continue
            w, h = rgb.get_width(), rgb.get_height()
            data = np.asanyarray(rgb.get_data()).reshape((h, w, 3))
            color = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)

            dw, dh = df.get_width(), df.get_height()
            raw = np.frombuffer(df.get_data(), dtype=np.uint16).reshape((dh, dw))
            depth = raw.astype(np.float32) * df.get_depth_scale()

            K = self.get_k()

            # 点云
            points = None
            if self._point_cloud_filter is not None:
                pcf = self._point_cloud_filter.process(aligned)
                if pcf is not None:
                    data = np.frombuffer(pcf.as_points_frame().get_data(), dtype=np.float32)
                    n = data.size // 6
                    if n > 0:
                        points = data.reshape(n, 6)  # x,y,z,r,g,b (r/g/b ∈ [0,255])

            return CameraFrame(color=color, depth=depth, K=K, points=points)

    def get_k(self) -> list[float]:
        if self._pipeline is None:
            raise RuntimeError("Camera not opened")
        intr = self._pipeline.get_camera_param().depth_intrinsic
        return [intr.fx, 0.0, intr.cx, 0.0, intr.fy, intr.cy, 0.0, 0.0, 1.0]


if __name__ == "__main__":
    cam = Camera()
    cam.open()

    device = cam._pipeline.get_device()

    # presets
    presets = device.get_available_preset_list()
    print(f"\nAvailable Presets ({len(presets)}):")
    for i in range(len(presets)):
        print(f"  [{i}] {presets[i]}")
    print(f"Current Preset: {device.get_current_preset_name()}")

    # properties
    n_props = device.get_support_property_count()
    print(f"\nSupported Properties ({n_props}):")
    from pyorbbecsdk import OBPermissionType, OBPropertyType
    for i in range(n_props):
        item = device.get_supported_property(i)
        perm = {OBPermissionType.PERMISSION_READ: "R", OBPermissionType.PERMISSION_WRITE: "W",
                OBPermissionType.PERMISSION_READ_WRITE: "RW"}.get(item.permission, "-")
        try:
            if item.type == OBPropertyType.OB_INT_PROPERTY:
                r = device.get_int_property_range(item.id)
                rng = f"[{r.min}, {r.max}, step={r.step}]"
            elif item.type == OBPropertyType.OB_FLOAT_PROPERTY:
                r = device.get_float_property_range(item.id)
                rng = f"[{r.min:.2f}, {r.max:.2f}, step={r.step:.2f}]"
            elif item.type == OBPropertyType.OB_BOOL_PROPERTY:
                rng = "[0, 1]"
            else:
                rng = f"type={item.type}"
        except Exception:
            rng = "N/A"
        print(f"  [{i:02d}] {item.name} (id={int(item.id)}) perm={perm} range={rng}")
        
    K = cam.get_k()
    print(f"K: {K}")

    for i in range(5):
        frame = cam.wait_for_frame(1000)
        if frame is None:
            print("timeout")
            continue
        print(f"[{i}] color: {frame.color.shape} {frame.color.dtype}  "
            f"depth: {frame.depth.shape} {frame.depth.dtype}  "
            f"range: [{frame.depth[frame.depth > 0].min():.0f}, {frame.depth.max():.0f}] mm")

    cam.close()
    print("\ndone")
