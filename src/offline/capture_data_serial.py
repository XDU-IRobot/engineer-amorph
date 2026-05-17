#!/usr/bin/env python3
"""
数据采集脚本：从 USB CDC 虚拟串口连续读取关节数据，实时写入 CSV 文件。
用于重力参数 b1, b2 的辨识回归。

下位机发送格式（每行）：
  stamp,q1,q2,q3,dq1,dq2,dq3,tau1,tau2,tau3\r\n

使用方法:
  python3 capture_data_serial.py [--port COM8] [--baud 115200] [--output data.csv]

操作流程:
  脚本启动后自动连续记录每一帧有效数据，按 Ctrl+C 退出并保存。
"""

import argparse
import csv
import os
import sys
from datetime import datetime

import serial


FIELDNAMES = ["stamp", "q1", "q2", "q3", "dq1", "dq2", "dq3", "tau1", "tau2", "tau3"]


def main():
    parser = argparse.ArgumentParser(description="从串口连续采集关节数据")
    parser.add_argument("-p", "--port", default="COM6", help="串口号，例如 COM3 或 /dev/ttyACM0")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="波特率（默认 115200）")
    default_output = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    parser.add_argument("-o", "--output", default=default_output, help="输出 CSV 文件路径")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] 无法打开串口 {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    csv_file = open(args.output, "w", newline="")
    writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
    writer.writeheader()

    print(f"[INFO] 已连接串口 {args.port}，波特率 {args.baud}")
    print(f"[INFO] 连续采集已启动，数据实时写入 {args.output}")
    print("[INFO] 按 Ctrl+C 停止采集并退出。")

    count = 0
    try:
        while True:
            try:
                raw = ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue

            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != 10:
                continue

            try:
                sample = {k: float(v) for k, v in zip(FIELDNAMES, parts)}
            except ValueError:
                continue

            writer.writerow(sample)
            csv_file.flush()  # 保证即使异常退出也不丢数据

            count += 1
            if count % 10 == 0:
                print(f"[{count}] q=({sample['q1']:.4f}, {sample['q2']:.4f}, {sample['q3']:.4f})  "
                      f"tau=({sample['tau1']:.4f}, {sample['tau2']:.4f}, {sample['tau3']:.4f})")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        csv_file.close()

    print(f"[INFO] 已保存 {count} 组数据到 {args.output}")


if __name__ == "__main__":
    main()
