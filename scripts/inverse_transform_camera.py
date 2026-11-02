#!/usr/bin/env python3
"""
手眼标定逆变换计算脚本
输入：base->camera 的变换 (x, y, z, roll, pitch, yaw)，单位为毫米/米和度
输出：camera->base 的逆变换 (x', y', z', roll', pitch', yaw')，单位为相同单位
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import sys

def inverse_transform(x, y, z, roll, pitch, yaw, degrees=True):
    """
    计算齐次变换矩阵的逆变换，并返回平移和 RPY 欧拉角。

    参数:
        x, y, z : 平移量
        roll, pitch, yaw : 欧拉角（绕固定轴 X-Y-Z 顺序）
        degrees : 布尔值，True 表示输入输出角度为度，False 为弧度

    返回:
        (inv_x, inv_y, inv_z, inv_roll, inv_pitch, inv_yaw)
    """
    # 平移向量
    t = np.array([x, y, z])

    # 创建旋转矩阵
    if degrees:
        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    else:
        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    R_mat = rot.as_matrix()

    # 逆变换：R_inv = R^T, t_inv = -R^T * t
    R_inv = R_mat.T
    t_inv = -R_inv @ t

    # 将逆旋转矩阵转回欧拉角
    rot_inv = R.from_matrix(R_inv)
    euler_inv = rot_inv.as_euler('xyz', degrees=degrees)

    return t_inv[0], t_inv[1], t_inv[2], euler_inv[0], euler_inv[1], euler_inv[2]

def main():
    # 支持命令行参数：python script.py x y z roll pitch yaw
    if len(sys.argv) == 7:
        try:
            x, y, z, roll, pitch, yaw = map(float, sys.argv[1:7])
        except ValueError:
            print("错误：请输入有效的数值。")
            sys.exit(1)
    else:
        # 交互式输入
        print("请输入 base->camera 的变换量（单位：毫米和度）：")
        try:
            x = float(input("x: "))
            y = float(input("y: "))
            z = float(input("z: "))
            roll = float(input("roll (deg): "))
            pitch = float(input("pitch (deg): "))
            yaw = float(input("yaw (deg): "))
        except ValueError:
            print("输入无效，请确保输入数值。")
            sys.exit(1)

    inv_x, inv_y, inv_z, inv_roll, inv_pitch, inv_yaw = inverse_transform(
        x, y, z, roll, pitch, yaw, degrees=True
    )

    print("\n逆变换 (camera->base):")
    print(f"平移 (x, y, z): {inv_x:.6f}, {inv_y:.6f}, {inv_z:.6f}")
    print(f"旋转 (roll, pitch, yaw) [deg]: {inv_roll:.6f}, {inv_pitch:.6f}, {inv_yaw:.6f}")

if __name__ == "__main__":
    main()