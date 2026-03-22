import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R_sci
import math

def draw_frame(ax, T, R_mat, length=80.0, lw=3, alpha=1.0, name=''):
    """在3D空间绘制一个局部坐标系"""
    dirs = R_mat @ np.eye(3) * length
    ax.quiver(T[0], T[1], T[2], dirs[0, 0], dirs[1, 0], dirs[2, 0], color='r', linewidth=lw, alpha=alpha)
    ax.quiver(T[0], T[1], T[2], dirs[0, 1], dirs[1, 1], dirs[2, 1], color='g', linewidth=lw, alpha=alpha)
    ax.quiver(T[0], T[1], T[2], dirs[0, 2], dirs[1, 2], dirs[2, 2], color='b', linewidth=lw, alpha=alpha)
    if name:
        ax.text(T[0], T[1], T[2] + length*0.2, name, color='black', alpha=alpha)
# def full_6dof_ik(tgt_x, tgt_y, tgt_z, Tgt_R):
#     """将前三周位置解与后三轴姿态代数解进行合成封装"""
#     a1 = ik_t1_f(tgt_x, tgt_y, tgt_z)
#     a2 = ik_t2_f(tgt_x, tgt_y, tgt_z)
#     a3 = ik_t3_f(tgt_x, tgt_y, tgt_z)
    
#     # 将求得的角度代回正运动学，获取手腕球心处由于前三个关节导致的当前旋转姿态
#     R03_val = fk_R03_f(a1, a2, a3)
    
#     # 解除前三轴影响，求出单纯留给后三轴的剩余旋转目标(W)
#     # R_wrist = R03^-1 * Tgt_R 
#     W = R03_val.T @ Tgt_R
    
#     # 将剩余旋转代入刚才得出的解析方程式
#     w11, w12, w13 = W[0,0], W[0,1], W[0,2]
#     w21, w22, w23 = W[1,0], W[1,1], W[1,2]
#     w31, w32, w33 = W[2,0], W[2,1], W[2,2]
    
#     c5 = np.clip(w11, -1.0, 1.0)
#     a5 = math.acos(c5)
    
#     # 若需要另一组姿态解可以是: a5_b = -a5, 然后按之前逻辑颠倒符号。在此只取一个固定结果
#     if abs(math.sin(a5)) < 1e-6:
#         a4 = 0.0 # 死锁态
#         if c5 > 0:
#             a6 = math.atan2(w32, w22)
#         else:
#             a6 = -math.atan2(w32, w22)
#     else:
#         a4 = math.atan2(w21, -w31)
#         a6 = math.atan2(w12, w13)
        
#     return [a1, a2, a3, a4, a5, a6]

# ==========================================
# 1. 定义符号变量 (Symbolic Definitions)
# ==========================================
x, y, z = sp.symbols('x y z', real=True)
l1, l2, dz = sp.symbols('l1 l2 dz', positive=True)
t1, t2, t3, t4, t5, t6 = sp.symbols('t1 t2 t3 t4 t5 t6', real=True)

print("正在构建并简化 6-DOF 符号解析表达式...\n")

# ==========================================
# 2. 符号化前三轴与位置 (Base & Arm)
# ==========================================
P0 = sp.Matrix([0, 0, 0])
P1 = sp.Matrix([0, 0, dz])

r2 = l1 * sp.cos(t2)
z2 = dz + l1 * sp.sin(t2)
P2 = sp.Matrix([r2 * sp.cos(t1), r2 * sp.sin(t1), z2])

r3 = r2 + l2 * sp.cos(t2 + t3)
z3 = z2 + l2 * sp.sin(t2 + t3)
P3 = sp.Matrix([r3 * sp.cos(t1), r3 * sp.sin(t1), z3])

# 前三个关节产生的中介底座到腕部末端的旋转姿态 (R03)
# Yaw 绕 Z 轴; Pitch 绕(已转动的) Y 轴。右手法则中绕正Y轴旋转会使原X轴下沉向负Z偏移。
# 为了满足“正角度等于向上抬起”，需要绕Y旋转负角度 -(t2 + t3)
R_z1 = sp.Matrix([
    [sp.cos(t1), -sp.sin(t1), 0],
    [sp.sin(t1),  sp.cos(t1), 0],
    [0,          0,           1]
])
a23 = -(t2 + t3)
R_y23 = sp.Matrix([
    [sp.cos(a23),  0, sp.sin(a23)],
    [0,            1, 0],
    [-sp.sin(a23), 0, sp.cos(a23)]
])
R03 = R_z1 * R_y23

# ==========================================
# 3. 符号化后三轴 (Spherical Wrist R-P-R)
# ==========================================
R_x4 = sp.Matrix([
    [1, 0, 0],
    [0, sp.cos(t4), -sp.sin(t4)],
    [0, sp.sin(t4),  sp.cos(t4)]
])
R_y5 = sp.Matrix([
    [sp.cos(t5),  0, sp.sin(t5)],
    [0,           1, 0],
    [-sp.sin(t5), 0, sp.cos(t5)]
])
R_x6 = sp.Matrix([
    [1, 0, 0],
    [0, sp.cos(t6), -sp.sin(t6)],
    [0, sp.sin(t6),  sp.cos(t6)]
])
R_wrist_fk = R_x4 * R_y5 * R_x6

# 整体正向运动学旋转矩阵
R06 = R03 * R_wrist_fk

# ==========================================
# 4. 符号推导逆运动学 (Elbow Up + RPR 解析封闭解)
# ==========================================
print("--- 符号推导 6-DOF 逆向运动学解析方程 ---")

# 4.1 位置核心解 (解三角形)
theta1_ik = sp.atan2(y, x)
r_val = sp.sqrt(x**2 + y**2)
z_val = z - dz
L_sq = r_val**2 + z_val**2
cos_t3 = (L_sq - l1**2 - l2**2) / (2 * l1 * l2)

# 限定取 Elbow Up: 即P2点比连接P1-P3的最短线更高
# 连杆2相对连杆1向下折叠，所以在此坐标系设置下 t3 为负
theta3_ik = -sp.acos(cos_t3)

k1 = l1 + l2 * sp.cos(theta3_ik)
k2 = l2 * sp.sin(theta3_ik)
theta2_ik = sp.atan2(z_val, r_val) - sp.atan2(k2, k1)

print("位置段 (Elbow UP):")
print(f"Yaw    (t1) = {theta1_ik}")
print(f"Pitch1 (t2) = {theta2_ik}")
print(f"Pitch2 (t3) = {theta3_ik}\n")

# 4.2 提取内回旋转逆解
print("姿态段 (Spherical Wrist R-P-R):")
print("令 W = R03(-t1, -t2, -t3)^T * R_target_06，其九个位置分量为 w_ij:")
print("Roll1  (t4) = atan2(w_21, -w_31)")
print("Pitch3 (t5) = acos(w_11)")
print("Roll2  (t6) = atan2(w_12, w_13)")

# ==========================================
# 5. 数值函数化与随机测试
# ==========================================
params = {l1: 408.83, l2: 490.0, dz: 40.0}
ik_t1_f = sp.lambdify((x, y, z), theta1_ik.subs(params), 'numpy')
ik_t2_f = sp.lambdify((x, y, z), theta2_ik.subs(params), 'numpy')
ik_t3_f = sp.lambdify((x, y, z), theta3_ik.subs(params), 'numpy')

fk_P_all = sp.lambdify((t1, t2, t3), 
                        [P0.subs(params), P1.subs(params), P2.subs(params), P3.subs(params)], 
                        'numpy')
fk_R03_f = sp.lambdify((t1, t2, t3), R03, 'numpy')
fk_R06_f = sp.lambdify((t1, t2, t3, t4, t5, t6), R06, 'numpy')

def full_6dof_ik(tgt_x, tgt_y, tgt_z, Tgt_R):
    """将前三周位置解与后三轴姿态代数解进行合成封装"""
    # 强制转换为 float，避免 Sympy 数值类型在后续计算中引起潜在的类型冲突
    a1 = float(ik_t1_f(tgt_x, tgt_y, tgt_z))
    a2 = float(ik_t2_f(tgt_x, tgt_y, tgt_z))
    a3 = float(ik_t3_f(tgt_x, tgt_y, tgt_z))
    
    # 将求得的角度代回正运动学，获取手腕球心处由于前三个关节导致的当前旋转姿态
    R03_val = np.array(fk_R03_f(a1, a2, a3), dtype=np.float64)
    
    # 解除前三轴影响，求出单纯留给后三轴的剩余旋转目标(W)
    W = R03_val.T @ Tgt_R
    
    w11, w12, w13 = W[0,0], W[0,1], W[0,2]
    w21, w22, w23 = W[1,0], W[1,1], W[1,2]
    w31, w32, w33 = W[2,0], W[2,1], W[2,2]
    
    # ==========================================================
    # 引入 PUMA560 经典的解耦解析法 (使用 atan2 而非 acos)
    # 依据 Rx-Ry-Rx 的特性: w12^2 + w13^2 = sin^2(a5)
    # ==========================================================
    s5_pos = math.sqrt(w12**2 + w13**2)
    
    # 【解法 1】：非翻转构型 (No-Flip)，假定 sin(a5) 为正
    a5_1 = math.atan2(s5_pos, w11)
    if s5_pos < 1e-6: # 奇异态 (死锁)
        a4_1 = 0.0 
        if w11 > 0:
            a6_1 = math.atan2(w32, w22)
        else:
            a6_1 = -math.atan2(w32, w22)
    else:
        a4_1 = math.atan2(w21, -w31)
        a6_1 = math.atan2(w12, w13)
        
    # 【解法 2】：翻转构型 (Flip)，假定 sin(a5) 为负（经典 PUMA 必算解）
    a5_2 = math.atan2(-s5_pos, w11)
    if s5_pos < 1e-6: # 奇异态下退化，与解法1一致
        a4_2 = 0.0
        if w11 > 0:
            a6_2 = math.atan2(w32, w22)
        else:
            a6_2 = -math.atan2(w32, w22)
    else:
        a4_2 = math.atan2(-w21, w31)
        a6_2 = math.atan2(-w12, -w13)
        
    # 你可以根据需求返回 a5 为正 或 a5 为负的解。
    # 这里默认返回第一组解，你也可以将代码扩充为返回两组解并选取最接近当前机械臂状态的一组。
    return [a1, a2, a3, a4_1, a5_1, a6_1]

# ---------- 生成一组随机可达测试案例 ----------
target_p = np.array([550.0, -200.0, 300.0])
target_orient = R_sci.random()
target_R = target_orient.as_matrix()

print("\n==========================================")
print(f"测试末端位置 Target XYZ: {target_p}")
print(f"测试末端姿态 Target Quat: {target_orient.as_quat().round(4)}")

# 求解计算
try:
    angles = full_6dof_ik(target_p[0], target_p[1], target_p[2], target_R)
    print("\n顺利抽出完整闭式逆解 (Elbow Up + Positive Pitch3):")
    labels = ["Yaw", "Pitch1", "Pitch2", "Roll1", "Pitch3", "Roll2"]
    for lbl, rad in zip(labels, angles):
        print(f" - {lbl:<7s} = {np.degrees(rad):>7.2f}°")
except Exception as e:
    print("坐标超出几何工作空间!", e)
    exit(1)
    
# 以代入解算出 FK，反向校验
test_R06 = fk_R06_f(*angles)
test_P   = np.array(fk_P_all(angles[0], angles[1], angles[2])).reshape(4,3)
calc_P3  = test_P[3, :]

pos_err = np.linalg.norm(calc_P3 - target_p)
rot_err = np.linalg.norm(test_R06 - target_R)

print(f"\n以结果验算正向运动学误差，位置残差: {pos_err:.2e},  旋转残差: {rot_err:.2e}")
if pos_err < 1e-5 and rot_err < 1e-5:
    print(">>>> 验证成功！完美完成 6-DOF 封闭解析！")
else:
    print(">>>> 验证存在异常误差！")

# ==========================================
# 6. 三维验证可视化
# ==========================================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title('Amorph 6-DOF Complete IK Verification (Elbow Up + Spherical Wrist)')

# 连线绘制连杆
ax.plot(test_P[:,0], test_P[:,1], test_P[:,2], '-o', color='gray', linewidth=5, markersize=8, label='Arm Links')

# 绘制基础坐标系 (半透明)
draw_frame(ax, np.zeros(3), np.eye(3), length=150.0, lw=2, alpha=0.3, name="Base")
# 绘制目标要求姿态 (加粗明亮线)
draw_frame(ax, target_p, target_R, length=120.0, lw=5, alpha=0.8, name="Target TCP")

# 绘制解算出的末端坐标系姿态 (用黄、青、紫色虚线覆盖上去验证重合)
calc_dirs = test_R06 @ np.eye(3) * 125.0
ax.quiver(calc_P3[0], calc_P3[1], calc_P3[2], calc_dirs[0,0], calc_dirs[1,0], calc_dirs[2,0], color='y', linestyle='dashed', linewidth=3)
ax.quiver(calc_P3[0], calc_P3[1], calc_P3[2], calc_dirs[0,1], calc_dirs[1,1], calc_dirs[2,1], color='c', linestyle='dashed', linewidth=3)
ax.quiver(calc_P3[0], calc_P3[1], calc_P3[2], calc_dirs[0,2], calc_dirs[1,2], calc_dirs[2,2], color='m', linestyle='dashed', linewidth=3, label="FK Frame")

# 标定轴视界
max_range = np.array([test_P[:,0].max()-test_P[:,0].min(), 
                        test_P[:,1].max()-test_P[:,1].min(), 
                        test_P[:,2].max()-test_P[:,2].min()]).max() / 2.0
mid_x = (test_P[:,0].max()+test_P[:,0].min()) * 0.5
mid_y = (test_P[:,1].max()+test_P[:,1].min()) * 0.5
mid_z = (test_P[:,2].max()+test_P[:,2].min()) * 0.5
ax.set_xlim(mid_x - max_range*1.5, mid_x + max_range*1.5)
ax.set_ylim(mid_y - max_range*1.5, mid_y + max_range*1.5)
ax.set_zlim(mid_z - max_range*1.5, mid_z + max_range*1.5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend(loc='upper right')

plt.show()
