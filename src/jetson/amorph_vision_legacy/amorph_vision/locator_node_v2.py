# 解决 Jetson 平台 libgomp.so.1 TLS 加载问题
import ctypes
try:
    ctypes.CDLL('/lib/aarch64-linux-gnu/libgomp.so.1', mode=ctypes.RTLD_GLOBAL)
except OSError:
    # 非 ARM64 平台或库不存在时忽略
    pass

# std
import os
import time
from typing import Any, Dict, List

# thirdparty
import cv2
import numpy as np
from ultralytics import YOLO  # type: ignore

# ros
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
import message_filters
import tf2_ros
from ament_index_python.packages import get_package_share_directory

# ros interfaces
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped

# local
from .pose_smoother import PoseSmoother
from .locator_backend import LocatorBackend
from .visualization import publish_masked_pointcloud, publish_image
from .publisher_pool import PublisherPool
from .tracker_sort_3d import Sort3DTracker

# profiler
# from viztracer import trace_and_save


class EnergyUnitLocator(Node):
    def __init__(self):
        super().__init__('energy_unit_locator')

        # 参数声明
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('erosion_kernel_size', 5)
        self.declare_parameter('erosion_iterations', 1)
        self.declare_parameter('publish_masked_pointcloud', False)
        self.declare_parameter('publish_annotated_image', False)
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('tracker_max_age', 15)
        self.declare_parameter('tracker_min_hits', 1)
        self.declare_parameter('tracker_mahalanobis_thresh', 15.0)
        self.declare_parameter('tracker_iou_weight', 0.4)

        # 获取参数
        model_path = self.get_parameter(
            'model_path').get_parameter_value().string_value
        if not model_path:
            pkg_share = get_package_share_directory('amorph_assets')
            model_path = os.path.join(
                pkg_share, 'assets', '251120_epoch335.pt')

        self.conf_thresh = self.get_parameter(
            'confidence_threshold').get_parameter_value().double_value
        self.erosion_kernel_size = self.get_parameter(
            'erosion_kernel_size').get_parameter_value().integer_value
        self.erosion_iterations = self.get_parameter(
            'erosion_iterations').get_parameter_value().integer_value
        self.publish_masked_pointcloud_enabled = self.get_parameter(
            'publish_masked_pointcloud').get_parameter_value().bool_value
        self.publish_annotated_image_enabled = self.get_parameter(
            'publish_annotated_image').get_parameter_value().bool_value
        smoothing_alpha = self.get_parameter(
            'smoothing_alpha').get_parameter_value().double_value
        tracker_max_age = self.get_parameter(
            'tracker_max_age').get_parameter_value().integer_value
        tracker_min_hits = self.get_parameter(
            'tracker_min_hits').get_parameter_value().integer_value
        tracker_mahalanobis_thresh = self.get_parameter(
            'tracker_mahalanobis_thresh').get_parameter_value().double_value
        tracker_iou_weight = self.get_parameter(
            'tracker_iou_weight').get_parameter_value().double_value

        # 加载yolo模型
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path, task='segment')
        self.get_logger().info('YOLO model loaded successfully')

        self.pose_smoother = PoseSmoother(alpha=smoothing_alpha)
        self.tracker = Sort3DTracker(
            max_age=max(1, int(tracker_max_age)),
            min_hits=max(1, int(tracker_min_hits)),
            mahalanobis_thresh=max(1e-3, float(tracker_mahalanobis_thresh)),
            iou_weight=max(0.0, min(1.0, float(tracker_iou_weight))),
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.publisher_pool = PublisherPool(self)
        self.bridge = CvBridge()

        # 创建订阅者
        self.intrinsics = None
        self.locator_backend = None
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.info_callback,
            10)

        # 图像 QoS: 最新优先 + BEST_EFFORT 可靠性（低延迟）
        image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.sub_rgb = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw', qos_profile=image_qos)
        self.sub_depth = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw', qos_profile=image_qos)

        # 同步RGB图和深度图
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth],
            queue_size=2,
            slop=0.033
        )
        self.ts.registerCallback(self.image_callback)

        # 创建统计 timer（每 1 秒运行一次）
        self.stats_timer = self.create_timer(1.0, self.stats_callback)

        # 性能统计
        self.frame_count = 0
        self.last_frame_detections_count = 0  # 前一帧检测到的目标数
        self.last_fps_time = time.time()
        self.last_frame_time = None
        self.total_inference_time = 0.0
        self.total_backend_time = 0.0
        self.processed_frames = 0

        self.get_logger().info('node started')

    def info_callback(self, msg):
        if self.intrinsics is None:
            if self.erosion_kernel_size is None or self.erosion_iterations is None:
                self.get_logger().warn('Locator backend params not ready yet', throttle_duration_sec=2.0)
                return

            self.intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5]
            }
            # 初始化定位后端
            self.locator_backend = LocatorBackend(
                intrinsics=self.intrinsics,
                erosion_kernel_size=int(self.erosion_kernel_size),
                erosion_iterations=int(self.erosion_iterations)
            )
            self.get_logger().info(
                f'Camera Intrinsics received: {self.intrinsics}')

    # @trace_and_save
    def image_callback(self, rgb_msg, depth_msg):
        if self.locator_backend is None:
            self.get_logger().warn('Waiting for camera intrinsics...', throttle_duration_sec=2.0)
            return
        if self.tracker is None:
            self.get_logger().warn('Tracker not initialized yet', throttle_duration_sec=2.0)
            return
        if self.bridge is None or self.tf_broadcaster is None or self.pose_smoother is None or self.intrinsics is None:
            self.get_logger().warn('Node runtime resources are not ready', throttle_duration_sec=2.0)
            return

        bridge = self.bridge
        tf_broadcaster = self.tf_broadcaster
        pose_smoother = self.pose_smoother
        intrinsics = self.intrinsics

        try:
            # 将 ROS 图像转换为 OpenCV 格式
            cv_rgb = bridge.imgmsg_to_cv2(
                rgb_msg, desired_encoding='bgr8')
            cv_depth = bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')  # 通常是 16UC1
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # 推理
        inference_start = time.time()
        results = self.model(cv_rgb, verbose=False, conf=self.conf_thresh) # type: ignore
        inference_time = time.time() - inference_start

        # 累积统计数据
        self.frame_count += 1
        self.processed_frames += 1
        self.total_inference_time += inference_time

        valid_detections: List[Dict[str, Any]] = []

        for result in results:
            if result.masks is None:
                continue

            masks = result.masks.data.cpu().numpy()
            boxes = result.boxes.data.cpu().numpy()

            # 显式二值化：TensorRT FP16 engine 输出的 mask 可能是连续 sigmoid 值
            # 而非干净的 0/1，直接 .astype(np.uint8) 会截断为全 0
            masks = (masks > 0.5).astype(np.float32)

            # TensorRT engine 的 mask 可能仍处于模型输入分辨率（含 letterbox 填充），
            # 需要先裁掉 padding 再 resize，否则 mask 会与深度图错位
            img_h, img_w = cv_rgb.shape[:2]
            mask_h, mask_w = masks.shape[1], masks.shape[2]
            img_aspect = img_w / img_h
            mask_aspect = mask_w / mask_h
            if abs(img_aspect - mask_aspect) > 0.01:
                # 宽高比不同，说明存在 letterbox 填充
                if img_aspect > mask_aspect:
                    # 上下有填充
                    effective_h = int(mask_w / img_aspect)
                    pad = (mask_h - effective_h) // 2
                    masks = masks[:, pad:pad + effective_h, :]
                else:
                    # 左右有填充
                    effective_w = int(mask_h * img_aspect)
                    pad = (mask_w - effective_w) // 2
                    masks = masks[:, :, pad:pad + effective_w]

            # 推理出来的mask大小可能和深度图不匹配，调整一下
            if masks.shape[1:] != cv_depth.shape:
                masks = np.array([cv2.resize(mask, (cv_depth.shape[1], cv_depth.shape[0]),
                                 interpolation=cv2.INTER_NEAREST) for mask in masks])

            frame_detections = 0  # 当前帧检测到的目标数

            # 标注、发布图像，调试用
            if self.publish_annotated_image_enabled:
                annotated_frame = result.plot()
                publish_image(self.publisher_pool, annotated_frame)

            for i, mask in enumerate(masks):
                # 使用后端处理mask
                backend_start = time.time()
                pose_result = self.locator_backend.process_mask(
                    mask, boxes[i], cv_depth)
                backend_time = time.time() - backend_start
                self.total_backend_time += backend_time

                if pose_result is None:
                    continue

                valid_detections.append({
                    'position': pose_result['position'],
                    'orientation': pose_result['orientation'],
                    'axis_direction': pose_result['axis_direction'],
                    'binary_mask': pose_result['binary_mask'],
                    'box': boxes[i][:4],
                })

        now = time.time()
        if self.last_frame_time is None:
            dt = 1.0 / 30.0
        else:
            dt = max(1.0 / 120.0, min(0.5, now - self.last_frame_time))
        self.last_frame_time = now

        assignments, removed_track_ids = self.tracker.update(valid_detections, dt=dt)
        for track_id in removed_track_ids:
            pose_smoother.reset(track_id)

        frame_detections = 0  # 当前帧已确认并发布的目标数
        for assignment in assignments:
            track_id = assignment.track_id
            det = valid_detections[assignment.detection_index]

            position = det['position']
            orientation = det['orientation']
            axis_direction = det['axis_direction']
            binary_mask = det['binary_mask']

            # 发布mask后的点云用于可视化
            if self.publish_masked_pointcloud_enabled:
                publish_masked_pointcloud(
                    self.publisher_pool,
                    binary_mask,
                    cv_rgb,
                    cv_depth,
                    intrinsics,
                    depth_msg.header
                )

            # 发布原始TF（使用稳定的 track_id）
            t = TransformStamped()
            t.header.stamp = depth_msg.header.stamp
            t.header.frame_id = depth_msg.header.frame_id
            t.child_frame_id = f'energy_unit_{track_id}'
            t.transform.translation.x = position[0]
            t.transform.translation.y = position[1]
            t.transform.translation.z = position[2]
            t.transform.rotation.x = orientation[0]
            t.transform.rotation.y = orientation[1]
            t.transform.rotation.z = orientation[2]
            t.transform.rotation.w = orientation[3]
            tf_broadcaster.sendTransform(t)

            # 计算并发布平滑后的TF（按 track_id 平滑）
            smoothed_pos, smoothed_ori = pose_smoother.update(
                track_id, position, orientation)

            t_smoothed = TransformStamped()
            t_smoothed.header.stamp = depth_msg.header.stamp
            t_smoothed.header.frame_id = depth_msg.header.frame_id
            t_smoothed.child_frame_id = f'energy_unit_{track_id}_smoothed'
            t_smoothed.transform.translation.x = smoothed_pos[0]
            t_smoothed.transform.translation.y = smoothed_pos[1]
            t_smoothed.transform.translation.z = smoothed_pos[2]
            t_smoothed.transform.rotation.x = smoothed_ori[0]
            t_smoothed.transform.rotation.y = smoothed_ori[1]
            t_smoothed.transform.rotation.z = smoothed_ori[2]
            t_smoothed.transform.rotation.w = smoothed_ori[3]
            tf_broadcaster.sendTransform(t_smoothed)

            if axis_direction is not None:
                frame_detections += 1

        # 更新前一帧检测计数
        self.last_frame_detections_count = frame_detections

    def stats_callback(self):
        """统计回调：每 1 秒输出一次性能统计"""
        if self.processed_frames == 0:
            return

        # 计算平均值
        avg_inference_time = (self.total_inference_time /
                              self.processed_frames) * 1000  # ms
        avg_backend_time = (self.total_backend_time /
                            self.processed_frames) * 1000  # ms
        fps = self.frame_count

        self.get_logger().debug(
            f'{self.last_frame_detections_count} detections, '
            f'{fps} FPS, '
            f'Inference={avg_inference_time:.1f}ms, '
            f'Backend={avg_backend_time:.1f}ms'
        )

        # 重置计数器
        self.frame_count = 0
        self.total_inference_time = 0.0
        self.total_backend_time = 0.0
        self.processed_frames = 0


def main(args=None):
    rclpy.init(args=args)
    node = EnergyUnitLocator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown() 会被lifecycle node自动管理


if __name__ == '__main__':
    main()