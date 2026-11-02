#!/usr/bin/env python3
"""ROS2 节点: RGB + Depth -> YOLO segment -> PCA -> ICP -> 3D SORT -> publish TF."""

from typing import cast

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
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
from amorph_vision.tracker_sort_3d import Sort3DTracker
from amorph_vision.visualization import (
    publish_masked_pointcloud, publish_yolo_annotation)

from viztracer import VizTracer


class EuLocatorNode(Node):
    def __init__(self):
        super().__init__('eu_locator_v3_node')

        self.declare_parameter('ply_path', '')
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.85)
        # 3D SORT: 轨迹丢失后保留帧数、确认前所需命中帧数
        self.declare_parameter('track_max_age', 15)
        self.declare_parameter('track_min_hits', 3)
        # 关联门限: 3D 马氏距离上限与 2D IoU 权重
        self.declare_parameter('track_mahalanobis_thresh', 15.0)
        self.declare_parameter('track_iou_weight', 0.4)
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        share_dir = get_package_share_directory('amorph_vision')
        self._ply_path = self.get_parameter('ply_path').value or f'{share_dir}/energy_unit.ply'
        self._model_path = self.get_parameter('model_path').value or f'{share_dir}/best.pt'

        self.intrinsics = None
        self.estimator = None
        self.bridge = CvBridge()
        self._last_stamp = None

        def int_param(name: str) -> int:
            return self.get_parameter(
                name).get_parameter_value().integer_value

        def double_param(name: str) -> float:
            return self.get_parameter(
                name).get_parameter_value().double_value

        self.tracker = Sort3DTracker(
            max_age=int_param('track_max_age'),
            min_hits=int_param('track_min_hits'),
            mahalanobis_thresh=double_param('track_mahalanobis_thresh'),
            iou_weight=double_param('track_iou_weight'))

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
        if self.estimator is None or self.intrinsics is None:
            self.get_logger().warn(
                'Waiting for camera_info...', throttle_duration_sec=5)
            return

        color = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')

        # 16UC1 (mm) 或 32FC1 (m) -> float32 mm
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

        stamp = rgb_msg.header.stamp
        frame_id = rgb_msg.header.frame_id
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9

        # 帧间隔用于卡尔曼预测, 用消息时间戳因此回放 bag 也正确
        if self._last_stamp is None:
            dt = 1.0 / 30.0
        else:
            dt = max(1.0 / 120.0, min(0.5, stamp_sec - self._last_stamp))
        self._last_stamp = stamp_sec

        # ultralytics 的返回类型标注包含 Tensor/Iterator, 这里收敛为 Results 列表
        results = cast(list, self.model(color, conf=conf, verbose=False))

        # 先对每个检测做位姿估计: 3D SORT 需要 3D 位置做关联
        detections = []
        # 与 publish_yolo_annotation 的检测顺序一一对应, 用于在图上显示 track id
        annotation_ids = []
        for r in results:
            if r.boxes is None or r.masks is None:
                continue
            for i in range(len(r.boxes)):
                annotation_ids.append(None)
                raw_mask = r.masks.data[i].cpu().numpy().astype(np.uint8)
                box = r.boxes.xyxy[i].tolist() + [
                    float(r.boxes.conf[i].item()),
                    float(r.boxes.cls[i].item())]

                pose = self.estimator.estimate(color, depth, raw_mask, box)
                if pose is None:
                    continue

                detections.append({
                    'position': pose['position'],
                    'orientation': pose['orientation'],
                    'mask': pose['binary_mask'],
                    'box': box,
                    'annotation_index': len(annotation_ids) - 1,
                })

        assignments, removed_ids = self.tracker.update(detections, dt=dt)
        for track_id in removed_ids:
            self.get_logger().info(
                f'track {track_id} dropped', throttle_duration_sec=2)

        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        for assignment in assignments:
            track_id = assignment.track_id
            detection = detections[assignment.detection_index]
            position = detection['position']
            orientation = detection['orientation']
            annotation_ids[detection['annotation_index']] = track_id

            # 发布该目标的 mask 点云, topic 按 track id 命名
            publish_masked_pointcloud(
                self.publisher_pool, detection['mask'], color, depth,
                self.intrinsics, header, str(track_id))

            # TF
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = frame_id
            t.child_frame_id = f'energy_unit_{track_id}'
            t.transform.translation.x = position[0]
            t.transform.translation.y = position[1]
            t.transform.translation.z = position[2]
            t.transform.rotation.x = orientation[0]
            t.transform.rotation.y = orientation[1]
            t.transform.rotation.z = orientation[2]
            t.transform.rotation.w = orientation[3]
            self.tf_broadcaster.sendTransform(t)

        # 发布 YOLO 标注供 foxglove 叠加显示（标签中包含 track id）
        publish_yolo_annotation(
            self.publisher_pool, results, header, track_ids=annotation_ids)

        self.get_logger().debug(
            f'Frame processed: {len(assignments)} published, '
            f'{len(detections)} detected',
            throttle_duration_sec=1)


def main(args=None):
    rclpy.init(args=args)
    node = EuLocatorNode()
    try:
        # with VizTracer(output_file="optional.json") as tracer:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
