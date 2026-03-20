import logging
import time

import numpy as np
import pyrealsense2 as rs


logger = logging.getLogger(__name__)

class T265:
    """RealSense T265追踪器"""

    def __init__(self, serial_number=None, start_retries=10, retry_interval=1.0):
        """初始化T265并支持短暂断连后的自动重试"""
        self.serial_number = serial_number
        self.start_retries = start_retries
        self.retry_interval = retry_interval
        self.pipeline = None
        self.config = None
        self._origin_pose = None
        self._last_raw_pose = None
        self._reset_on_next_frame = False
        self._start_pipeline(start_retries, retry_interval)

    @staticmethod
    def _quat_normalize(q):
        q = np.asarray(q, dtype=np.float64)
        norm = np.linalg.norm(q)
        if norm == 0:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        return q / norm

    @staticmethod
    def _quat_conjugate(q):
        q = np.asarray(q, dtype=np.float64)
        return q * np.array([1.0, -1.0, -1.0, -1.0], dtype=np.float64)

    @staticmethod
    def _quat_multiply(q1, q2):
        q1 = np.asarray(q1, dtype=np.float64)
        q2 = np.asarray(q2, dtype=np.float64)

        w1, v1 = q1[0], q1[1:]
        w2, v2 = q2[0], q2[1:]

        w = w1 * w2 - np.dot(v1, v2)
        v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
        return np.concatenate(([w], v))

    @staticmethod
    def _quat_to_rotation_matrix(q):
        q = T265._quat_normalize(q)
        w, x, y, z = q
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def _quat_to_euler(self, q):
        """四元数转欧拉角，返回(yaw, pitch, roll)，单位为弧度"""
        q = self._quat_normalize(q)
        w, x, y, z = q
        # Yaw (Z), Pitch (Y), Roll (X)
        t0 = +2.0 * (w * z + x * y)
        t1 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * x + y * z)
        t4 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t3, t4)
        return yaw, pitch, roll

    def _euler_to_quat(self, yaw, pitch, roll):
        """欧拉角(yaw, pitch, roll)转四元数，输入为弧度，返回(w, x, y, z)"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return np.array([w, x, y, z], dtype=np.float64)

    def _apply_origin_transform(self, raw_pose):
        if self._origin_pose is None:
            return raw_pose

        origin_t = np.asarray(self._origin_pose['translation'], dtype=np.float64)
        raw_t = np.asarray(raw_pose['translation'], dtype=np.float64)
        rel_t = raw_t - origin_t

        # 只消除yaw的相对变化，pitch和roll保持当前
        origin_q = self._quat_normalize(self._origin_pose['rotation'])
        raw_q = self._quat_normalize(raw_pose['rotation'])
        origin_yaw, _, _ = self._quat_to_euler(origin_q)
        raw_yaw, raw_pitch, raw_roll = self._quat_to_euler(raw_q)
        rel_yaw = raw_yaw - origin_yaw
        # 新四元数：rel_yaw + 当前pitch/roll
        rel_q = self._euler_to_quat(rel_yaw, raw_pitch, raw_roll)

        return {
            'translation': tuple(float(v) for v in rel_t),
            'rotation': tuple(float(v) for v in rel_q),
            'tracker_confidence': raw_pose['tracker_confidence'],
            'mapper_confidence': raw_pose['mapper_confidence'],
        }

    def _find_pose_device_serial(self):
        ctx = rs.context()
        devices = ctx.query_devices()

        for device in devices:
            for sensor in device.query_sensors():
                for profile in sensor.get_stream_profiles():
                    if profile.stream_type() == rs.stream.pose:
                        if device.supports(rs.camera_info.serial_number):
                            return device.get_info(rs.camera_info.serial_number)
                        return None
        return None

    def _find_device(self):
        """按配置序列号或首个可用pose设备查找device对象。"""
        ctx = rs.context()
        devices = ctx.query_devices()

        # 指定序列号时优先精确匹配
        if self.serial_number:
            for device in devices:
                if device.supports(rs.camera_info.serial_number):
                    serial = device.get_info(rs.camera_info.serial_number)
                    if serial == self.serial_number:
                        return device
            return None

        # 未指定序列号时找首个支持pose流的设备
        for device in devices:
            for sensor in device.query_sensors():
                for profile in sensor.get_stream_profiles():
                    if profile.stream_type() == rs.stream.pose:
                        return device
        return None

    def _start_pipeline(self, retries, retry_interval):
        last_error = None

        for attempt in range(1, retries + 1):
            try:
                self.pipeline = rs.pipeline()
                self.config = rs.config()

                device_serial = self.serial_number or self._find_pose_device_serial()
                if self.serial_number and not device_serial:
                    raise RuntimeError(f"T265 with serial {self.serial_number} not found")
                if not self.serial_number and not device_serial:
                    raise RuntimeError("No device with pose stream found")

                self.config.enable_device(device_serial)
                # 某些pyrealsense2版本/固件组合下，强制format会导致"Couldn't resolve requests"
                self.config.enable_stream(rs.stream.pose)
                self.pipeline.start(self.config)
                logger.info("T265 initialized successfully (serial=%s)", device_serial)
                return
            except RuntimeError as e:
                last_error = e
                logger.warning(
                    "T265 start failed (%s/%s): %s",
                    attempt,
                    retries,
                    e,
                )
                time.sleep(retry_interval)

        raise RuntimeError(f"T265 initialization failed after {retries} retries: {last_error}")
    
    def get_pose_data(self):
        """获取姿态数据（已转换至ROS坐标系：前X+ 左Y+ 上Z+）"""
        if self.pipeline is None:
            raise RuntimeError("T265 pipeline is not initialized")

        frames = self.pipeline.wait_for_frames()
        pose_frame = frames.first_or_default(rs.stream.pose)
        if pose_frame:
            pose_data = pose_frame.as_pose_frame().get_pose_data()
            
            # --- 坐标系转换：T265 -> ROS ---
            # T265 原生: X右(+), Y上(+), Z后(+)
            # ROS 标准: X前(-Z_raw), Y左(-X_raw), Z上(+Y_raw)
            raw_pose = {
                'translation': (
                    -pose_data.translation.z,
                    -pose_data.translation.x,
                    pose_data.translation.y, 
                ),
                'rotation': (
                    pose_data.rotation.w, 
                    -pose_data.rotation.z,
                    -pose_data.rotation.x,
                    pose_data.rotation.y, 
                ),
                'tracker_confidence': int(pose_data.tracker_confidence),
                'mapper_confidence': int(pose_data.mapper_confidence),
            }

            self._last_raw_pose = raw_pose
            if self._reset_on_next_frame:
                self._origin_pose = raw_pose
                self._reset_on_next_frame = False

            pose = self._apply_origin_transform(raw_pose)
            tx, ty, tz = pose['translation']
            rw, rx, ry, rz = pose['rotation']

            return {
                'x': tx,
                'y': ty,
                'z': tz,
                'w': rw,
                'rx': rx,
                'ry': ry,
                'rz': rz,
                'tracker_confidence': pose['tracker_confidence'],
                'mapper_confidence': pose['mapper_confidence'],
            }
        return None

    def soft_reset(self):
        """软件归零：将当前平移和yaw设为新原点，pitch和roll保持原样。"""
        if self._last_raw_pose is not None:
            # 只保存平移和yaw
            pose = self._last_raw_pose.copy()
            q = self._quat_normalize(pose['rotation'])
            yaw, _, _ = self._quat_to_euler(q)
            # 保存yaw到_origin_pose，pitch/roll不参与重置
            pose['rotation'] = tuple(q)
            pose['yaw'] = yaw
            self._origin_pose = pose
            self._reset_on_next_frame = False
            logger.info("T265 soft reset: current translation and yaw set as origin")
        else:
            # 尚未拿到有效pose时，延迟到下一帧完成归零
            self._reset_on_next_frame = True
            logger.info("T265 soft reset requested: waiting for next valid pose frame")

    def hard_reset(self, reconnect=True):
        """硬复位：通过pipeline stop再start实现，不调用hardware_reset。"""
        if self.pipeline is None:
            logger.warning("T265 hard reset: pipeline not initialized")
            return
        try:
            self.pipeline.stop()
            logger.info("T265 pipeline stopped for hard reset")
        except Exception as e:
            logger.warning(f"T265 pipeline stop failed during hard reset: {e}")

        # 清空软件原点状态
        self._origin_pose = None
        self._last_raw_pose = None
        self._reset_on_next_frame = False

        if reconnect:
            try:
                self.pipeline.start(self.config)
                logger.info("T265 pipeline restarted for hard reset")
            except Exception as e:
                logger.error(f"T265 pipeline restart failed during hard reset: {e}")
        else:
            logger.info("T265 hard reset complete (reconnect disabled)")
    
    def stop(self):
        """停止管道"""
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except RuntimeError:
                pass