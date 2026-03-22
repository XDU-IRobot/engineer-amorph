import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import math

def rpr_fk(theta4, theta5, theta6):
    """
    后三轴(Roll-Pitch-Roll) 正向运动学
    对应内旋: 绕X轴转theta4, 绕新Y轴转theta5, 绕最新X轴转theta6
    """
    R4 = R.from_euler('x', theta4, degrees=False).as_matrix()
    R5 = R.from_euler('y', theta5, degrees=False).as_matrix()
    R6 = R.from_euler('x', theta6, degrees=False).as_matrix()
    
    return R4 @ R5 @ R6

def rpr_ik(R_target):
    """
    后三轴(Roll-Pitch-Roll) 逆向运动学求封闭解
    R_target 为给定的四元数目标计算出的 3x3 旋转矩阵
    """
    r11, r12, r13 = R_target[0, 0], R_target[0, 1], R_target[0, 2]
    r21, r22, r23 = R_target[1, 0], R_target[1, 1], R_target[1, 2]
    r31, r32, r33 = R_target[2, 0], R_target[2, 1], R_target[2, 2]
    
    # Pitch是由r11=cos(theta5)独立决定的，取一个正解一个负解
    # 为了避免浮点数精度超界溢出
    c5 = np.clip(r11, -1.0, 1.0)
    theta5_sol1 = math.acos(c5)
    theta5_sol2 = -math.acos(c5)
    
    # 检查是否处于万向节死锁(Gimbal Lock)的情况, 即Pitch刚好是0度或180度
    if abs(np.sin(theta5_sol1)) < 1e-6:
        # 当sin(theta5) == 0 时, Roll 1 和 Roll 2在同一根轴上，存在无限解
        # 此处我们强行固定 theta4 为 0，把所有的偏差交给最后一个 Roll (theta6) 计算
        theta4_sol1 = 0.0
        theta4_sol2 = 0.0
        
        # 通过代数展开，此时: r22 = cos(t4 +- t6), r32 = sin(t4 +- t6)
        if c5 > 0: # theta5 是 0
            theta6_sol1 = math.atan2(r32, r22)
            theta6_sol2 = math.atan2(r32, r22)
        else:      # theta5 是 180 (Pi)
            theta6_sol1 = -math.atan2(r32, r22)
            theta6_sol2 = -math.atan2(r32, r22)
            
    else:
        # 正常状态，非奇点。根据矩阵代数：
        # r21 = sin(t4)*sin(t5)
        # r31 = -cos(t4)*sin(t5)
        # r12 = sin(t5)*sin(t6)
        # r13 = sin(t5)*cos(t6)
        
        # 解组 1
        s5_1 = math.sin(theta5_sol1)
        theta4_sol1 = math.atan2(r21, -r31) if s5_1 > 0 else math.atan2(-r21, r31)
        theta6_sol1 = math.atan2(r12, r13) if s5_1 > 0 else math.atan2(-r12, -r13)
        
        # 解组 2
        s5_2 = math.sin(theta5_sol2)
        theta4_sol2 = math.atan2(r21, -r31) if s5_2 > 0 else math.atan2(-r21, r31)
        theta6_sol2 = math.atan2(r12, r13) if s5_2 > 0 else math.atan2(-r12, -r13)

    return (theta4_sol1, theta5_sol1, theta6_sol1), (theta4_sol2, theta5_sol2, theta6_sol2)

def draw_frame(ax, T, R, length=1.0, lw=2, alpha=1.0, name=''):
    """在3D空间绘制一个局部坐标系"""
    origins = np.array([T, T, T])
    # 提取XYZ这三个方向的单位向量并乘以长度
    dirs = R @ np.eye(3) * length
    
    # 绘制箭头: X是红，Y是绿，Z是蓝
    ax.quiver(T[0], T[1], T[2], dirs[0, 0], dirs[1, 0], dirs[2, 0], color='r', linewidth=lw, alpha=alpha)
    ax.quiver(T[0], T[1], T[2], dirs[0, 1], dirs[1, 1], dirs[2, 1], color='g', linewidth=lw, alpha=alpha)
    ax.quiver(T[0], T[1], T[2], dirs[0, 2], dirs[1, 2], dirs[2, 2], color='b', linewidth=lw, alpha=alpha)
    if name:
        ax.text(T[0], T[1], T[2] + length*0.2, name, color='black', alpha=alpha)

def main():
    np.set_printoptions(precision=4, suppress=True)

    # 1. 随机生成一个目标的末端朝向，以四元数或欧拉角的方式均可
    # 为测试丰富性，这里直接生成一组随机的欧拉角转化为目标姿态
    target_orient = R.random()
    R_target = target_orient.as_matrix()
    target_quat = target_orient.as_quat() # x, y, z, w
    
    print("\n==============================")
    print("目标末端四元数 [x, y, z, w]:", [round(q, 4) for q in target_quat])
    print("目标旋转矩阵:\n", R_target)
    
    # 2. 调用IK解析求得三种角度
    sol1, sol2 = rpr_ik(R_target)
    print("\n------------------------------")
    print(f"求得解组 1: Roll={math.degrees(sol1[0]):.2f}°, Pitch={math.degrees(sol1[1]):.2f}°, Roll2={math.degrees(sol1[2]):.2f}°")
    print(f"求得解组 2: Roll={math.degrees(sol2[0]):.2f}°, Pitch={math.degrees(sol2[1]):.2f}°, Roll2={math.degrees(sol2[2]):.2f}°")
    
    # 3. 把解带回FK验证是否能复现那个姿态
    R_calc1 = rpr_fk(*sol1)
    R_calc2 = rpr_fk(*sol2)
    err1 = np.linalg.norm(R_target - R_calc1)
    err2 = np.linalg.norm(R_target - R_calc2)
    print("\n------------------------------")
    print(f"解1 代回正运动学运算，复现矩阵残差: {err1:.2e}")
    print(f"解2 代回正运动学运算，复现矩阵残差: {err2:.2e}")
    
    if err1 < 1e-6 and err2 < 1e-6:
        print("\n验证成功! FK完全复现该目标姿态！(测试完美通过)")
    else:
        print("\n验证失败! 计算出偏差较大!")
        return

    # 4. 可视化
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    base_pos = np.zeros(3)

    # 绘制基础坐标系(半透明)
    draw_frame(ax, base_pos, np.eye(3), length=1.2, lw=1, alpha=0.3, name="Base")
    
    # 绘制目标坐标系(加粗实线)
    draw_frame(ax, base_pos, R_target, length=1.5, lw=4, alpha=1.0, name="Target")
    
    # 绘制复现出来的坐标系(用黄色虚线表示X，青色虚线Y，紫色虚线Z覆盖)
    dirs = R_calc1 @ np.eye(3) * 1.5
    ax.quiver(0,0,0, dirs[0,0], dirs[1,0], dirs[2,0], color='y', linestyle='dashed', linewidth=2, label="Calculated X")
    ax.quiver(0,0,0, dirs[0,1], dirs[1,1], dirs[2,1], color='c', linestyle='dashed', linewidth=2, label="Calculated Y")
    ax.quiver(0,0,0, dirs[0,2], dirs[1,2], dirs[2,2], color='m', linestyle='dashed', linewidth=2, label="Calculated Z")

    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Amorph 3-DOF Wrist (R-P-R) Verification")
    ax.legend(loc='upper left')
    
    plt.show()

if __name__ == "__main__":
    main()
