import math
from luma.core.render import canvas

class AttitudeIndicator:
    def __init__(self, device):
        self.device = device
        self.width = device.width
        self.height = device.height
        self.center_x = self.width // 2
        self.center_y = self.height // 2
        # 仪表盘半径稍稍缩小，给边缘留出显示空间
        self.radius = 24 

    def quaternion_to_euler(self, q):
        w, x, y, z = q
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

    def rotate_point(self, x, y, angle):
        s, c = math.sin(angle), math.cos(angle)
        return x * c - y * s, x * s + y * c

    def draw(self, q):
        roll, pitch, yaw = self.quaternion_to_euler(q)
        pitch = -pitch
        roll = -roll
        
        with canvas(self.device) as draw:
            # 地平线
            # 增加地平线长度至 200，确保覆盖 128 宽的屏幕
            pitch_offset = (pitch / (math.pi / 2)) * 40 # 俯仰灵敏度系数
            
            # 计算长直线的端点坐标 (相对中心)
            lx0, ly0 = -100, pitch_offset
            lx1, ly1 = 100, pitch_offset
            
            # 旋转并平移
            rx0, ry0 = self.rotate_point(lx0, ly0, roll)
            rx1, ry1 = self.rotate_point(lx1, ly1, roll)
            
            # 绘制地平线
            draw.line((rx0 + self.center_x, ry0 + self.center_y, 
                       rx1 + self.center_x, ry1 + self.center_y), fill="white", width=1)

            # 中央静态十字准星
            gap = 5
            length = 15
            draw.line((self.center_x - length, self.center_y, self.center_x - gap, self.center_y), fill="white")
            draw.line((self.center_x + gap, self.center_y, self.center_x + length, self.center_y), fill="white")
            draw.line((self.center_x, self.center_y - gap, self.center_x, self.center_y + gap), fill="white")

            # 航向指示器
            # 箭头始终指向Yaw=0
            target_angle = yaw 
            
            # 绘制一条从中心指向箭头的虚线 (辅助视觉)
            tip_dist = self.radius + 10
            tx = tip_dist * math.sin(target_angle) + self.center_x
            ty = -tip_dist * math.cos(target_angle) + self.center_y
            # 简单模拟虚线：画一段短线
            line_start_x = (self.radius + 2) * math.sin(target_angle) + self.center_x
            line_start_y = -(self.radius + 2) * math.cos(target_angle) + self.center_y
            draw.line((line_start_x, line_start_y, tx, ty), fill="white")

            # 绘制大尺寸实心箭头
            # 箭头三角形的三个顶点 (尖端向上)
            arrow_w = 6  # 箭头宽度
            arrow_h = 10 # 箭头高度
            p1 = (0, -arrow_h)
            p2 = (-arrow_w, 0)
            p3 = (arrow_w, 0)
            
            rotated_arrow = []
            for px, py in [p1, p2, p3]:
                # 旋转箭头自身，使其指向圆心外侧
                rx, ry = self.rotate_point(px, py, target_angle)
                # 平移到圆周位置
                rotated_arrow.append((rx + tx, ry + ty))
            
            # 填充箭头
            draw.polygon(rotated_arrow, outline="white", fill="white")

            # 仪表盘装饰外圈
            draw.ellipse((self.center_x - self.radius, self.center_y - self.radius,
                          self.center_x + self.radius, self.center_y + self.radius), 
                         outline="white")

            # 在底部留出黑色背景区域显示数字
            draw.rectangle((0, 52, 128, 64), fill="black")
            info = f"R:{math.degrees(roll):3.0f}° P:{math.degrees(pitch):3.0f}° Y:{math.degrees(yaw):3.0f}°"
            draw.text((10, 53), info, fill="white")

# test
if __name__ == "__main__":
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306
    import time

    # 初始化屏幕
    serial = i2c(port=1, address=0x3C)
    device = ssd1306(serial)
    
    indicator = AttitudeIndicator(device)

    try:
        angle = 0
        while True:
            # 模拟一个动态变化的四元数 (绕 Y 轴旋转模拟俯仰)
            angle += 0.05
            pitch_sim = math.sin(angle) * 0.5  # 模拟俯仰 ±28度
            roll_sim = math.cos(angle) * 0.3   # 模拟滚转 ±17度
            
            # 简单的欧拉角转四元数 (用于演示)
            cy, sy = math.cos(0 * 0.5), math.sin(0 * 0.5) # Yaw = 0
            cp, sp = math.cos(pitch_sim * 0.5), math.sin(pitch_sim * 0.5)
            cr, sr = math.cos(roll_sim * 0.5), math.sin(roll_sim * 0.5)

            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy

            # 调用组件绘制
            indicator.draw([qw, qx, qy, qz])
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        pass