"""
双点深度测距节点

对深度图中两个可配置像素坐标附近的矩形窗口取平均深度，
通过两个 sensor_msgs/Range 话题发布测距结果。
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Range

# 红外辐射类型常量
RADIATION_TYPE_INFRARED = 1


class TwoPointRange(Node):
    """订阅深度图，对两点区域取平均深度并以 Range 消息发布"""

    def __init__(self):
        super().__init__('two_point_range')

        # 参数声明 — 第1点像素坐标
        self.declare_parameter('point1_x', 320)
        self.declare_parameter('point1_y', 240)

        # 参数声明 — 第2点像素坐标
        self.declare_parameter('point2_x', 640)
        self.declare_parameter('point2_y', 240)

        # 参数声明 — 取平均的窗口大小（正方形边长，像素）
        self.declare_parameter('window_size', 10)

        # 参数声明 — Range 消息字段
        self.declare_parameter('radiation_type', RADIATION_TYPE_INFRARED)
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('frame_id', 'camera_depth_frame')

        # 参数声明 — Range 消息的 field_of_view (弧度)
        self.declare_parameter('field_of_view', 0.01)

        # 获取参数
        self.point1_x = self.get_parameter('point1_x').value
        self.point1_y = self.get_parameter('point1_y').value
        self.point2_x = self.get_parameter('point2_x').value
        self.point2_y = self.get_parameter('point2_y').value
        self.window_size = self.get_parameter('window_size').value
        self.radiation_type = self.get_parameter('radiation_type').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.frame_id = self.get_parameter('frame_id').value
        self.field_of_view = self.get_parameter('field_of_view').value

        # CvBridge
        self.bridge = CvBridge()

        # QoS — 匹配现有节点模式：最新优先 + BEST_EFFORT
        image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # 订阅深度图
        self.sub_depth = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            image_qos
        )

        # 发布者 — 两个 Range 话题
        self.pub_range0 = self.create_publisher(Range, '~/range_0', 10)
        self.pub_range1 = self.create_publisher(Range, '~/range_1', 10)

        # 参数变更回调
        self.add_on_set_parameters_callback(self._on_param_change)

        # 日志
        self.get_logger().info(
            f'TwoPointRange node started.\n'
            f'  point1: ({self.point1_x}, {self.point1_y})\n'
            f'  point2: ({self.point2_x}, {self.point2_y})\n'
            f'  window_size: {self.window_size}\n'
            f'  frame_id: {self.frame_id}\n'
            f'  min_range: {self.min_range}, max_range: {self.max_range}'
        )

    def _on_param_change(self, params):
        """动态参数变更回调"""
        for param in params:
            if param.name == 'point1_x':
                self.point1_x = param.value
            elif param.name == 'point1_y':
                self.point1_y = param.value
            elif param.name == 'point2_x':
                self.point2_x = param.value
            elif param.name == 'point2_y':
                self.point2_y = param.value
            elif param.name == 'window_size':
                self.window_size = param.value
            elif param.name == 'radiation_type':
                self.radiation_type = param.value
            elif param.name == 'min_range':
                self.min_range = param.value
            elif param.name == 'max_range':
                self.max_range = param.value
            elif param.name == 'frame_id':
                self.frame_id = param.value
            elif param.name == 'field_of_view':
                self.field_of_view = param.value

        self.get_logger().info(
            f'Parameters updated: point1=({self.point1_x},{self.point1_y}), '
            f'point2=({self.point2_x},{self.point2_y}), '
            f'window_size={self.window_size}'
        )
        return rclpy.parameter.SetParametersResult(successful=True)

    def depth_callback(self, msg: Image):
        """深度图回调：提取两点窗口平均深度并发布 Range"""
        try:
            # CvBridge 将 ROS Image 转 numpy (16-bit 毫米)
            depth_img: np.ndarray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}', throttle_duration_sec=5.0)
            return

        h, w = depth_img.shape
        half = self.window_size // 2

        # 处理两个点
        range_msg_0 = self._compute_range(
            depth_img, w, h,
            self.point1_x, self.point1_y, half,
            msg.header
        )
        range_msg_1 = self._compute_range(
            depth_img, w, h,
            self.point2_x, self.point2_y, half,
            msg.header
        )

        # 发布
        if range_msg_0 is not None:
            self.pub_range0.publish(range_msg_0)
        if range_msg_1 is not None:
            self.pub_range1.publish(range_msg_1)

    def _compute_range(
        self,
        depth_img: np.ndarray,
        img_width: int,
        img_height: int,
        cx: int,
        cy: int,
        half_win: int,
        header
    ):
        """计算单点窗口的平均深度，返回 Range 消息或 None"""
        # 窗口边界（clamp 到图像范围）
        x1 = max(0, cx - half_win)
        x2 = min(img_width, cx + half_win + 1)
        y1 = max(0, cy - half_win)
        y2 = min(img_height, cy + half_win + 1)

        # 提取窗口区域
        region = depth_img[y1:y2, x1:x2].astype(np.float32)

        # 有效深度（> 0）
        valid = region[region > 0]

        # 有效像素阈值：至少需要窗口面积 / 4 个有效点
        window_area = (x2 - x1) * (y2 - y1)
        min_valid = max(1, window_area // 4)

        if valid.size < min_valid:
            self.get_logger().warn(
                f'Point ({cx},{cy}): insufficient valid depth pixels '
                f'({valid.size}/{window_area}), skipping',
                throttle_duration_sec=2.0
            )
            return None

        # 平均深度 (毫米 → 米)
        mean_depth_mm = float(np.mean(valid))
        mean_depth_m = mean_depth_mm / 1000.0

        # 构造 Range 消息
        range_msg = Range()
        range_msg.header.stamp = header.stamp
        range_msg.header.frame_id = self.frame_id
        range_msg.radiation_type = self.radiation_type
        range_msg.field_of_view = self.field_of_view
        range_msg.min_range = self.min_range
        range_msg.max_range = self.max_range
        range_msg.range = mean_depth_m

        return range_msg


def main(args=None):
    rclpy.init(args=args)
    node = TwoPointRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
