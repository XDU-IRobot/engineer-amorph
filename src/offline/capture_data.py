#!/usr/bin/env python3
"""
数据采集节点：订阅 /joint_states，手动按 Enter 键采集一组数据，保存为 CSV 文件。
用于重力参数 b1, b2 的辨识回归。

使用方法:
  ros2 run <pkg> capture_data  (如果以 ROS 包方式安装)
  或直接:  python3 capture_data.py [--output data.csv]

操作流程:
  1. 将机械臂置于某个静止姿态
  2. 按 Enter 采集当前关节角度和力矩
  3. 重复以上步骤，采集足够多的数据点
  4. 按 q + Enter 退出并保存
"""

import argparse
import csv
import os
import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# 动力学模型对应的前 3 个关节名称（按顺序）
JOINT_NAMES = ["j1_l", "j2_l", "j3_l"]


class DataCaptureNode(Node):
    def __init__(self, output_path: str):
        super().__init__("data_capture_node")
        self.output_path = output_path

        self._latest_msg: JointState | None = None
        self._lock = threading.Lock()
        self._samples: list[dict] = []

        self.sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        self.get_logger().info(
            f"数据采集节点已启动，监听 /joint_states，数据将保存至 {output_path}"
        )
        self.get_logger().info("按 Enter 采集一组数据，输入 q 退出并保存。")

    # ---- ROS 回调 ----
    def _joint_state_cb(self, msg: JointState):
        with self._lock:
            self._latest_msg = msg

    # ---- 采集一帧 ----
    def capture_once(self) -> bool:
        """采集当前帧数据，返回 True 表示成功。"""
        with self._lock:
            msg = self._latest_msg

        if msg is None:
            self.get_logger().warn("尚未收到 /joint_states 消息，请确认话题是否发布。")
            return False

        # 建立 name -> index 映射
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        # 检查所需关节是否都存在
        for jn in JOINT_NAMES:
            if jn not in name_to_idx:
                self.get_logger().error(f"关节 '{jn}' 不在 joint_states 中，当前: {msg.name}")
                return False

        indices = [name_to_idx[jn] for jn in JOINT_NAMES]

        positions = [msg.position[i] for i in indices]
        velocities = [msg.velocity[i] for i in indices] if msg.velocity else [0.0] * len(indices)
        efforts = [msg.effort[i] for i in indices] if msg.effort else [0.0] * len(indices)

        sample = {
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "q1": positions[0],
            "q2": positions[1],
            "q3": positions[2],
            "dq1": velocities[0],
            "dq2": velocities[1],
            "dq3": velocities[2],
            "tau1": efforts[0],
            "tau2": efforts[1],
            "tau3": efforts[2],
        }
        self._samples.append(sample)

        n = len(self._samples)
        self.get_logger().info(
            f"[#{n}] q=({sample['q1']:.4f}, {sample['q2']:.4f}, {sample['q3']:.4f})  "
            f"tau=({sample['tau1']:.4f}, {sample['tau2']:.4f}, {sample['tau3']:.4f})"
        )
        return True

    # ---- 保存 CSV ----
    def save_csv(self):
        if not self._samples:
            self.get_logger().warn("没有采集到数据，不保存文件。")
            return

        fieldnames = ["stamp", "q1", "q2", "q3", "dq1", "dq2", "dq3", "tau1", "tau2", "tau3"]
        os.makedirs(os.path.dirname(self.output_path) or ".", exist_ok=True)

        with open(self.output_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._samples)

        self.get_logger().info(
            f"已保存 {len(self._samples)} 组数据到 {self.output_path}"
        )


def input_loop(node: DataCaptureNode):
    """在单独线程中读取终端输入。"""
    while rclpy.ok():
        try:
            line = input()
        except EOFError:
            break
        if line.strip().lower() == "q":
            node.save_csv()
            rclpy.shutdown()
            break
        node.capture_once()


def main():
    parser = argparse.ArgumentParser(description="手动采集 joint_states 数据")
    default_output = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    parser.add_argument("-o", "--output", default=default_output, help="输出 CSV 文件路径")
    args = parser.parse_args()

    rclpy.init()
    node = DataCaptureNode(args.output)

    # 在后台线程中处理键盘输入
    input_thread = threading.Thread(target=input_loop, args=(node,), daemon=True)
    input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_csv()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
