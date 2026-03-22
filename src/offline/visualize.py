import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
from scipy.spatial.transform import Rotation as R_sci
import numpy as np
import math
import sys
import termios
import tty
import select

def solve_ik(x, y, z, Tgt_R):
    l1, l2, dz = 408.83, 490.0, 40.0
    
    # 1. 位置解 (Elbow Up)
    a1 = math.atan2(y, x)
    r_val = math.sqrt(x**2 + y**2)
    z_val = z - dz
    L_sq = r_val**2 + z_val**2
    cos_t3 = (L_sq - l1**2 - l2**2) / (2 * l1 * l2)
    cos_t3 = np.clip(cos_t3, -1.0, 1.0)
    
    a3 = -math.acos(cos_t3) # Elbow Up
    k1 = l1 + l2 * math.cos(a3)
    k2 = l2 * math.sin(a3)
    a2 = math.atan2(z_val, r_val) - math.atan2(k2, k1)
    
    # 2. 姿态解 (解除前三轴影响)
    a23 = -(a2 + a3)
    R_z1 = np.array([
        [math.cos(a1), -math.sin(a1), 0],
        [math.sin(a1),  math.cos(a1), 0],
        [0,            0,             1]
    ])
    R_y23 = np.array([
        [math.cos(a23),  0, math.sin(a23)],
        [0,              1, 0],
        [-math.sin(a23), 0, math.cos(a23)]
    ])
    R03 = R_z1 @ R_y23
    W = R03.T @ Tgt_R # 剩余旋转矩阵
    
    # 3. 后三轴解析
    c5 = np.clip(W[0,0], -1.0, 1.0)
    a5 = math.acos(c5)
    if abs(math.sin(a5)) < 1e-6:
        a4, a6 = 0.0, math.atan2(W[2,1], W[1,1])
    else:
        a4 = math.atan2(W[1,0], -W[2,0])
        a6 = math.atan2(W[0,1], W[0,2])
    
    return [a1, a2, a3, a4, a5, a6]

class IKTeleopNode(Node):
    def __init__(self):
        super().__init__('ik_teleop_node')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        # 初始目标状态 (单位: mm 和 弧度)
        self.target_p = np.array([550.0, 0.0, 400.0])
        self.target_euler = np.array([0.0, 0.0, 0.0]) # r, p, y
        
        self.msg_header = "--- IK Teleop 控制面板 ---\n" \
                          "移动: W/S (X+-) | A/D (Y+-) | Q/E (Z+-)\n" \
                          "旋转: U/J (Roll) | I/K (Pitch) | O/L (Yaw)\n" \
                          "CTRL+C 退出"
        print(self.msg_header)

        # 启动定时器发布状态
        self.timer = self.create_timer(0.05, self.publish_state)

    def publish_state(self):
        # 计算 IK
        tgt_R = R_sci.from_euler('xyz', self.target_euler).as_matrix()
        try:
            angles = solve_ik(self.target_p[0], self.target_p[1], self.target_p[2], tgt_R)
            
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['j1_l', 'j2_l', 'j3_l', 'j4_l', 'j5_l', 'j6_l', 'gripper_joint_l_l']
            js.position = [float(a) for a in angles] + [0.0]
            js.position[5] = -js.position[5]
            js.position[4] = -js.position[4]
            js.position[3] = -js.position[3]
            self.joint_pub.publish(js)

            ps = PoseStamped()
            ps.header.frame_id = "base_link" # 假设基坐标系叫这个
            ps.header.stamp = js.header.stamp
            ps.pose.position.x = self.target_p[0] / 1000.0
            ps.pose.position.y = self.target_p[1] / 1000.0
            ps.pose.position.z = self.target_p[2] / 1000.0
            q = R_sci.from_matrix(tgt_R).as_quat()
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.target_pub.publish(ps)
            
        except Exception as e:
            pass

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = IKTeleopNode()

    try:
        while rclpy.ok():
            # 使用 select 实现非阻塞键盘读取
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                # 步进值
                dp, dr = 10.0, math.radians(5.0)
                
                if key == 'w': node.target_p[0] += dp
                elif key == 's': node.target_p[0] -= dp
                elif key == 'a': node.target_p[1] -= dp
                elif key == 'd': node.target_p[1] += dp
                elif key == 'q': node.target_p[2] += dp
                elif key == 'e': node.target_p[2] -= dp
                elif key == 'u': node.target_euler[0] += dr
                elif key == 'j': node.target_euler[0] -= dr
                elif key == 'i': node.target_euler[1] += dr
                elif key == 'k': node.target_euler[1] -= dr
                elif key == 'o': node.target_euler[2] += dr
                elif key == 'l': node.target_euler[2] -= dr
                elif key == '\x03': # Ctrl+C
                    break
            
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()