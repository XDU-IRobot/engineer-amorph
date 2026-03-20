#!/usr/bin/env python3

import serial
import time
import logging
import colorlog
import traceback
from scipy.spatial.transform import Rotation

from packet_builder import PacketBuilder
from t265 import T265
from comm_struct import CartesianCustomControllerCommand
from button import Button


def setup_logger():
    """配置彩色日志输出"""
    handler = colorlog.StreamHandler()
    handler.setFormatter(
        colorlog.ColoredFormatter(
            '[%(asctime)s]%(log_color)s %(levelname)s %(reset)s %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S',
            log_colors={
                'DEBUG': 'cyan',
                'INFO': 'green',
                'WARNING': 'yellow',
                'ERROR': 'red',
                'CRITICAL': 'bold_red',
            },
        )
    )

    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.handlers.clear()
    root_logger.addHandler(handler)


setup_logger()
logger = logging.getLogger(__name__)

class Serial:
    """串口通信类"""
    
    def __init__(self, port, baudrate=115200, timeout=1):
        """初始化串口"""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            logger.info(f"Serial port opened: {port} @ {baudrate} baud")
        except Exception as e:
            logger.error(f"Failed to open serial port {port}: {e}")
            raise
    
    def write(self, data):
        """发送数据"""
        if isinstance(data, (bytearray, bytes)):
            self.ser.write(data)
        else:
            self.ser.write(bytes(data))
    
    def close(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Serial port closed")


def main():
    """主程序"""
    device = None
    serial_port = None
    reconnect_count = 0
    error_count = 0
    sent_count_window = 0
    sent_count_total = 0
    stats_window_start = time.monotonic()
    packet_builder = PacketBuilder()
    
    try:
        # 初始化设备
        serial_port = Serial("/dev/ttyAMA1", 115200)
        logger.info("Starting main loop...")

        # 初始化按钮
        button_a = Button(25)
        button_b = Button(8)
        button_reset = Button(7)

        while True:
            try:
                if device is None:
                    device = T265(start_retries=5, retry_interval=1.0)

                loop_start_time = time.time()

                # 获取姿态数据
                pose = device.get_pose_data()
                if pose is None:
                    continue

                command = CartesianCustomControllerCommand()
                command.is_cartesian = True
                command.ok = pose['tracker_confidence'] == 3
                command.btn_a = button_a.is_pressed()
                command.btn_b = button_b.is_pressed()
                command.translation_xyz[0] = pose['x']
                command.translation_xyz[1] = pose['y']
                command.translation_xyz[2] = pose['z']
                command.rotation_wxyz[0] = pose['w']
                command.rotation_wxyz[1] = pose['rx']
                command.rotation_wxyz[2] = pose['ry']
                command.rotation_wxyz[3] = pose['rz']

                if button_reset.is_pressed():
                    logger.info("Reset button pressed: Soft resetting T265")
                    device.hard_reset()

                r = Rotation.from_quat([pose['rx'], pose['ry'], pose['rz'], pose['w']])
                euler_deg = r.as_euler('xyz', degrees=True)

                debug_str = "\n".join([
                    "Pose data:",
                    f"  Translation: x={pose['x']:.3f} m, y={pose['y']:.3f} m, z={pose['z']:.3f} m",
                    f"  Rotation (quaternion): w={pose['w']:.3f}, x={pose['rx']:.3f}, y={pose['ry']:.3f}, z={pose['rz']:.3f}",
                    f"  Rotation (Euler angles): roll={euler_deg[0]:.1f}°, pitch={euler_deg[1]:.1f}°, yaw={euler_deg[2]:.1f}°",
                    f"  Tracker confidence: {pose['tracker_confidence']}, Mapper confidence: {pose['mapper_confidence']}",
                ])
                logger.debug(debug_str)
                logger.debug(f"Button : reset={button_reset.is_pressed()}, A={button_a.is_pressed()}, B={button_b.is_pressed()}")

                tx_packet = packet_builder.build(bytearray(command))
                logger.debug(f"Payload ({len(bytearray(command))} B): %s", " ".join(f"{b:02x}" for b in bytearray(command)))
                logger.debug(f"TX ({len(tx_packet)} B): %s", " ".join(f"{b:02x}" for b in tx_packet))

                serial_port.write(tx_packet)
                sent_count_window += 1
                sent_count_total += 1

                now = time.monotonic()
                window_elapsed = now - stats_window_start
                if window_elapsed >= 1.0:
                    rate_hz = sent_count_window / window_elapsed
                    logger.info(
                        "status: tx=%d, total=%d, rate=%.1fHz, reconnect=%d, error=%d",
                        sent_count_window,
                        sent_count_total,
                        rate_hz,
                        reconnect_count,
                        error_count,
                    )
                    sent_count_window = 0
                    stats_window_start = now

                # 控制循环频率为略小于30ms，保证发送频率尽量高
                elapsed_time = time.time() - loop_start_time
                sleep_time = 0.035 - elapsed_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

            except RuntimeError as e:
                error_count += 1
                reconnect_count += 1
                logger.warning(f"Camera unavailable or runtime error: {e}, retry in 1s...")
                traceback.print_exc()
                if device:
                    device.stop()
                    device = None
                time.sleep(1)
            except Exception as e:
                error_count += 1
                reconnect_count += 1
                logger.warning(f"Camera unavailable or runtime error: {e}, retry in 1s...")
                traceback.print_exc()
                if device:
                    device.stop()
                    device = None
                time.sleep(1)

    except RuntimeError as e:
        logger.error(f"RealSense error during initialization: {e}")
        traceback.print_exc()
    except Exception as e:
        logger.error(f"Exception: {e}")
        traceback.print_exc()
    finally:
        if device:
            device.stop()
        if serial_port:
            serial_port.close()
        logger.info("Program terminated")


if __name__ == "__main__":
    main()