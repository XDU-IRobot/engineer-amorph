# 解决 Jetson 平台 libgomp.so.1 TLS 加载问题
import ctypes
try:
    ctypes.CDLL('/lib/aarch64-linux-gnu/libgomp.so.1', mode=ctypes.RTLD_GLOBAL)
except OSError:
    # 非 ARM64 平台或库不存在时忽略
    pass

"""
Energy Unit Locator Node v2

使用 ultralytics YOLO + BoT-SORT 跟踪器进行多目标检测和跟踪。

关键特性:
- YOLO 模型进行目标检测和分割
- ultralytics.track() 方法集成 BoT-SORT 跟踪器
- 深度图 + 分割掩码进行3D位姿估计
- PoseSmoother 对位姿进行平滑处理
- 通过 TF 发布原始和平滑后的位姿

配置参数:
- model_path: YOLO模型路径
- confidence_threshold: 检测置信度阈值
- erosion_kernel_size: 腐蚀核大小
- erosion_iterations: 腐蚀迭代次数
- publish_masked_pointcloud: 是否发布mask后的点云
- publish_annotated_image: 是否发布标注图像
- smoothing_alpha: 位姿平滑系数
- tracking_persist: 是否在帧间保持跟踪器状态
- tracking_tracker: 跟踪器类型 (botsort, bytetrack, etc.)
"""

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
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
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

# profiler
# from viztracer import trace_and_save


class EnergyUnitLocator(LifecycleNode):
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
        self.declare_parameter('tracking_tracker', 'botsort.yaml')

        # 这些将在 configure 阶段初始化
        self.model = None
        self.conf_thresh = None
        self.erosion_kernel_size = None
        self.erosion_iterations = None
        self.publish_masked_pointcloud_enabled = None
        self.publish_annotated_image_enabled = None
        self.pose_smoother = None
        self.locator_backend = None
        self.tracking_tracker = None
        self.tf_broadcaster = None
        self.publisher_pool = None
        self.bridge = None
        self.intrinsics = None
        self.sub_info = None
        self.sub_rgb = None
        self.sub_depth = None
        self.ts = None
        
        self.tracking_persist = False

        # 节点激活状态标志
        self.is_active = False

        # 统计 timer
        self.stats_timer = None

        # 性能统计
        self.frame_count = 0
        self.last_frame_detections_count = 0  # 前一帧检测到的目标数
        self.last_fps_time = time.time()
        self.last_frame_time = None
        self.total_inference_time = 0.0
        self.total_backend_time = 0.0
        self.processed_frames = 0

        self.get_logger().info('node created (unconfigured)')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """配置阶段：加载模型、创建资源"""
        self.get_logger().info('configuring...')

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
        self.tracking_tracker = self.get_parameter(
            'tracking_tracker').get_parameter_value().string_value

        # 加载yolo模型
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        try:
            self.model = YOLO(model_path, task='segment')
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            return TransitionCallbackReturn.FAILURE

        self.pose_smoother = PoseSmoother(alpha=smoothing_alpha)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.publisher_pool = PublisherPool(self)
        self.bridge = CvBridge()

        # 创建订阅者
        self.intrinsics = None
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

        self.get_logger().info('Configuration complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """激活阶段：开始处理"""
        self.get_logger().info('activating...')

        # 创建统计 timer（每 1 秒运行一次）
        self.stats_timer = self.create_timer(1.0, self.stats_callback)
        
        self.tracking_persist = False   # 每次激活时重置跟踪器状态，确保不受之前状态影响

        if self.pose_smoother:
            self.pose_smoother.reset()
        self.last_frame_time = None

        self.is_active = True
        self.get_logger().info('node activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """停用阶段：停止处理"""
        self.get_logger().info('deactivating...')

        # 销毁统计 timer
        if self.stats_timer:
            self.destroy_timer(self.stats_timer)
            self.stats_timer = None

        self.is_active = False
        self.get_logger().info('node deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """清理阶段：释放资源"""
        self.get_logger().info('cleaning up...')

        # 确保 timer 已销毁
        if self.stats_timer:
            self.destroy_timer(self.stats_timer)
            self.stats_timer = None

        # 清理订阅者
        if self.ts:
            destroy = getattr(self.ts, 'destroy', None)
            if callable(destroy):
                destroy()
            self.ts = None
        if self.sub_rgb:
            self.sub_rgb.unregister()
            self.sub_rgb = None
        if self.sub_depth:
            self.sub_depth.unregister()
            self.sub_depth = None
        if self.sub_info:
            self.destroy_subscription(self.sub_info)
            self.sub_info = None

        # 清理其他资源
        self.model = None
        if self.pose_smoother:
            self.pose_smoother.reset()
        self.pose_smoother = None
        self.locator_backend = None
        self.tf_broadcaster = None
        self.publisher_pool = None
        self.bridge = None
        self.intrinsics = None

        # 重置统计数据
        self.frame_count = 0
        self.last_frame_detections_count = 0
        self.total_inference_time = 0.0
        self.total_backend_time = 0.0
        self.processed_frames = 0
        self.last_frame_time = None

        self.get_logger().info('cleanup complete')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """关闭阶段：最后的清理"""
        self.get_logger().info('shutting down...')

        # 再次确保所有资源都被释放
        if self.stats_timer:
            self.destroy_timer(self.stats_timer)
            self.stats_timer = None

        return TransitionCallbackReturn.SUCCESS

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
        # 只在 active 状态下处理图像
        if not self.is_active:
            return

        if self.locator_backend is None:
            self.get_logger().warn('Waiting for camera intrinsics...', throttle_duration_sec=2.0)
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

        # 推理 + 跟踪
        inference_start = time.time()
        results = self.model.track(
            cv_rgb,
            verbose=False,
            conf=self.conf_thresh,
            persist=self.tracking_persist,
            tracker=self.tracking_tracker
        )  # type: ignore
        inference_time = time.time() - inference_start
        
        if not self.tracking_persist:
            self.get_logger().info('Tracker state reset')
            self.tracking_persist = True  # 下一帧继续保持跟踪状态

        # 累积统计数据
        self.frame_count += 1
        self.processed_frames += 1
        self.total_inference_time += inference_time

        frame_detections = 0  # 当前帧已确认并发布的目标数

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

            # 标注、发布图像，调试用
            if self.publish_annotated_image_enabled:
                annotated_frame = result.plot()
                publish_image(self.publisher_pool, annotated_frame)

            # 获取track ID (ultralytics track API 在 result.boxes.id 中)
            track_ids = result.boxes.id.cpu().numpy() if result.boxes.id is not None else None

            for i, mask in enumerate(masks):
                # 使用后端处理mask
                backend_start = time.time()
                pose_result = self.locator_backend.process_mask(
                    mask, boxes[i], cv_depth)
                backend_time = time.time() - backend_start
                self.total_backend_time += backend_time

                if pose_result is None:
                    continue

                # 获取对应的track_id
                track_id = int(track_ids[i]) if track_ids is not None and i < len(track_ids) else None

                # 如果没有track_id，跳过（可能是新创建的track，需要稳定后再发布）
                if track_id is None:
                    continue

                position = pose_result['position']
                orientation = pose_result['orientation']
                axis_direction = pose_result['axis_direction']
                binary_mask = pose_result['binary_mask']

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

                # 发布原始TF（使用 ultralytics 提供的 track_id）
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

        self.get_logger().info(
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
