#!/usr/bin/env python3

import serial
import time
import logging
import colorlog
import traceback
import threading
import queue
from scipy.spatial.transform import Rotation

from packet_builder import PacketBuilder
from t265 import T265
from comm_struct import CartesianCustomControllerCommand
from button import Button
from oled_ui import AttitudeIndicator


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


class OLEDDisplayThread:
    """OLED显示的异步线程类"""
    
    def __init__(self, device, indicator, update_interval=0.05):
        """
        初始化OLED显示线程
        
        Args:
            device: OLED设备对象
            indicator: AttitudeIndicator实例
            update_interval: 更新间隔时间（秒）
        """
        self.device = device
        self.indicator = indicator
        self.update_interval = update_interval
        self.quaternion_queue = queue.Queue(maxsize=2)  # 只保留最新的数据
        self.running = False
        self.thread = None
    
    def update(self, quaternion):
        """
        更新姿态数据（非阻塞）
        
        Args:
            quaternion: 四元数 [w, x, y, z]
        """
        try:
            # 清空队列，只保留最新的数据
            while not self.quaternion_queue.empty():
                try:
                    self.quaternion_queue.get_nowait()
                except queue.Empty:
                    break
            self.quaternion_queue.put_nowait(quaternion)
        except queue.Full:
            pass  # 队列满了，忽略这次更新
    
    def _display_loop(self):
        """显示线程的主循环"""
        last_update_time = 0
        
        while self.running:
            try:
                # 控制更新频率
                current_time = time.time()
                if current_time - last_update_time < self.update_interval:
                    time.sleep(0.001)
                    continue
                
                # 从队列获取最新的姿态数据
                try:
                    quaternion = self.quaternion_queue.get_nowait()
                    self.indicator.draw(quaternion)
                    last_update_time = current_time
                except queue.Empty:
                    time.sleep(0.001)
                    continue
                    
            except Exception as e:
                logger.warning(f"OLED display error: {e}")
                time.sleep(0.1)
    
    def start(self):
        """启动显示线程"""
        if self.thread is None or not self.thread.is_alive():
            self.running = True
            self.thread = threading.Thread(target=self._display_loop, daemon=True)
            self.thread.start()
            logger.info("OLED display thread started")
    
    def stop(self):
        """停止显示线程"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
            logger.info("OLED display thread stopped")


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

        # 初始化OLED显示
        from luma.core.interface.serial import i2c
        from luma.oled.device import ssd1306
        serial_i2c = i2c(port=1, address=0x3C)
        oled_device = ssd1306(serial_i2c)
        attitude_indicator = AttitudeIndicator(oled_device)
        oled_thread = OLEDDisplayThread(oled_device, attitude_indicator, update_interval=0.05)
        oled_thread.start()
        logger.info("OLED display initialized")

        while True:
            try:
                if device is None:
                    device = T265(start_retries=5, retry_interval=1.0)

                loop_start_time = time.time()

                # 获取姿态数据
                pose = device.get_pose_data()
                if pose is None:
                    continue

                # 异步更新OLED显示姿态（非阻塞）
                quaternion = [pose['w'], pose['rx'], pose['ry'], pose['rz']]
                oled_thread.update(quaternion)

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
        # 停止OLED显示线程
        if 'oled_thread' in locals():
            oled_thread.stop()
        if device:
            device.stop()
        if serial_port:
            serial_port.close()
        logger.info("Program terminated")


if __name__ == "__main__":
    main()