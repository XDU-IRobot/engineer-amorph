#!/usr/bin/env python3
"""ROS2 节点: RGB + Depth → YOLO → PCA → ICP → PoseStamped + TF."""

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
import tf2_ros
from ultralytics import YOLO

from amorph_vision.pose_estimator import PoseEstimator
from amorph_vision.publisher_pool import PublisherPool
from amorph_vision.visualization import publish_masked_pointcloud


class EuLocatorNode(Node):
    def __init__(self):
        super().__init__('eu_locator_v3_node')

        self.declare_parameter('ply_path', '')
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.85)
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        share_dir = get_package_share_directory('amorph_vision')
        self._ply_path = self.get_parameter('ply_path').value or f'{share_dir}/energy_unit.ply'
        self._model_path = self.get_parameter('model_path').value or f'{share_dir}/best.pt'

        self.intrinsics = None
        self.estimator = None
        self.bridge = CvBridge()

        self.get_logger().info(f'Loading YOLO model: {self._model_path}')
        self.model = YOLO(self._model_path)

        self.rgb_sub = Subscriber(
            self, Image, self.get_parameter('rgb_topic').value)
        self.depth_sub = Subscriber(
            self, Image, self.get_parameter('depth_topic').value)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.camera_info_cb, 10)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.05)
        self.sync.registerCallback(self.frame_callback)

        self.pose_l_pub = self.create_publisher(PoseStamped, '~/pose_l', 10)
        self.pose_r_pub = self.create_publisher(PoseStamped, '~/pose_r', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.publisher_pool = PublisherPool(self)

        self.get_logger().info('node ready')

    def camera_info_cb(self, msg: CameraInfo):
        if self.intrinsics is not None:
            return
        K = msg.k
        self.intrinsics = {'fx': K[0], 'fy': K[4], 'cx': K[2], 'cy': K[5]}
        self.estimator = PoseEstimator(self._ply_path, self.intrinsics)
        self.get_logger().info(f'Estimator ready: fx={K[0]:.1f} fy={K[4]:.1f}')

    def frame_callback(self, rgb_msg: Image, depth_msg: Image):
        if self.estimator is None:
            self.get_logger().warn(
                'Waiting for camera_info...', throttle_duration_sec=5)
            return

        color = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')

        # 16UC1 (mm) 或 32FC1 (m) → float32 mm
        if depth_msg.encoding == '16UC1':
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, '16UC1').astype(np.float32)
        elif depth_msg.encoding == '32FC1':
            depth = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1') * 1000.0
        else:
            self.get_logger().error(
                f'Unsupported depth encoding: {depth_msg.encoding}')
            return

        conf = self.get_parameter('conf_threshold').value
        results = self.model(color, conf=conf, verbose=False)

        stamp = rgb_msg.header.stamp
        frame_id = rgb_msg.header.frame_id

        # 构建 header 用于点云发布
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        # 收集所有检测结果
        detections = []
        for r in results:
            if r.boxes is None or r.masks is None:
                continue
            for i in range(len(r.boxes)):
                cls_id = int(r.boxes.cls[i].item())
                raw_mask = r.masks.data[i].cpu().numpy().astype(np.uint8)
                box = r.boxes.xyxy[i].tolist() + [
                    float(r.boxes.conf[i].item()), float(cls_id)]
                # box[3] = y2 (底边), 越大越靠下
                detections.append(
                    (box[3], cls_id, raw_mask, box))

        if not detections:
            return

        # 按底边 y 从大到小排序: 从下到上, 取前两个目标
        detections.sort(key=lambda d: d[0], reverse=True)
        top_two = detections[:2]

        # 按 bbox 中心 x 坐标确定左右: x 小的为 l, x 大的为 r
        top_two.sort(key=lambda d: (d[3][0] + d[3][2]) / 2.0)

        for side, (_, cls_id, raw_mask, box) in zip(['l', 'r'], top_two):
            child_frame_id = f'energy_unit_{side}'

            # 发布选中目标的 mask 点云
            publish_masked_pointcloud(
                self.publisher_pool, raw_mask, color, depth,
                self.intrinsics, header, child_frame_id)

            pose = self.estimator.estimate(color, depth, raw_mask, box)
            if pose is not None:
                # PoseStamped
                ps = PoseStamped()
                ps.header.stamp = stamp
                ps.header.frame_id = frame_id
                ps.pose.position.x = pose['position'][0]
                ps.pose.position.y = pose['position'][1]
                ps.pose.position.z = pose['position'][2]
                ps.pose.orientation.x = pose['orientation'][0]
                ps.pose.orientation.y = pose['orientation'][1]
                ps.pose.orientation.z = pose['orientation'][2]
                ps.pose.orientation.w = pose['orientation'][3]
                self.pose_l_pub.publish(ps) if side == 'l' else self.pose_r_pub.publish(ps)

                # TF
                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = frame_id
                t.child_frame_id = child_frame_id
                t.transform.translation.x = pose['position'][0]
                t.transform.translation.y = pose['position'][1]
                t.transform.translation.z = pose['position'][2]
                t.transform.rotation.x = pose['orientation'][0]
                t.transform.rotation.y = pose['orientation'][1]
                t.transform.rotation.z = pose['orientation'][2]
                t.transform.rotation.w = pose['orientation'][3]
                self.tf_broadcaster.sendTransform(t)

        self.get_logger().debug(
            f'Frame processed: {len(detections)} detections, '
            f'selected {len(top_two)} targets',
            throttle_duration_sec=1)


def main(args=None):
    rclpy.init(args=args)
    node = EuLocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
