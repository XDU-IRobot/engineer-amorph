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

# thirdparty
import cv2
import numpy as np
from ultralytics import YOLO  # type: ignore

# ros
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
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

        # 这些将在 configure 阶段初始化
        self.model = None
        self.conf_thresh = None
        self.erosion_kernel_size = None
        self.erosion_iterations = None
        self.publish_masked_pointcloud_enabled = None
        self.publish_annotated_image_enabled = None
        self.pose_smoother = None
        self.locator_backend = None
        self.tf_broadcaster = None
        self.publisher_pool = None
        self.bridge = None
        self.intrinsics = None
        self.sub_info = None
        self.sub_rgb = None
        self.sub_depth = None
        self.ts = None

        # 节点激活状态标志
        self.is_active = False

        # 统计 timer
        self.stats_timer = None

        # 性能统计
        self.frame_count = 0
        self.last_frame_detections_count = 0  # 前一帧检测到的目标数
        self.last_fps_time = time.time()
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
            pkg_share = get_package_share_directory('energy_unit_locator')
            model_path = os.path.join(
                pkg_share, 'models', '251120_epoch335.engine')

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

        # 加载yolo模型
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        try:
            self.model = YOLO(model_path, task='segment')
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            return TransitionCallbackReturn.FAILURE

        # 创建位姿平滑器
        self.pose_smoother = PoseSmoother(alpha=smoothing_alpha)

        # 创建 tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 创建发布器池
        self.publisher_pool = PublisherPool(self)

        # 创建 CV Bridge
        self.bridge = CvBridge()

        # 创建订阅者
        self.intrinsics = None
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.info_callback,
            10)

        self.sub_rgb = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw')
        self.sub_depth = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.image_callback)

        self.get_logger().info('Configuration complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """激活阶段：开始处理"""
        self.get_logger().info('activating...')

        # 创建统计 timer（每 1 秒运行一次）
        self.stats_timer = self.create_timer(1.0, self.stats_callback)

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
            self.ts.destroy()
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
            self.intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5]
            }
            # 初始化定位后端
            self.locator_backend = LocatorBackend(
                intrinsics=self.intrinsics,
                erosion_kernel_size=self.erosion_kernel_size,
                erosion_iterations=self.erosion_iterations
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

        frame_start_time = time.time()

        try:
            # 将 ROS 图像转换为 OpenCV 格式
            cv_rgb = self.bridge.imgmsg_to_cv2(
                rgb_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(
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

                # 提取结果
                position = pose_result['position']
                orientation = pose_result['orientation']
                axis_direction = pose_result['axis_direction']
                radius_bottom = pose_result['radius_bottom']
                radius_top = pose_result['radius_top']
                raw_position = pose_result['raw_position']
                binary_mask = pose_result['binary_mask']

                # 统计当前帧检测到的目标
                if axis_direction is not None:
                    frame_detections += 1

                # 发布mask后的点云用于可视化
                if self.publish_masked_pointcloud_enabled:
                    publish_masked_pointcloud(
                        self.publisher_pool,
                        binary_mask,
                        cv_rgb,
                        cv_depth,
                        self.intrinsics,
                        depth_msg.header
                    )

                # 发布原始TF
                t = TransformStamped()
                t.header.stamp = depth_msg.header.stamp
                t.header.frame_id = depth_msg.header.frame_id
                t.child_frame_id = f'energy_unit_{i}'
                t.transform.translation.x = position[0]
                t.transform.translation.y = position[1]
                t.transform.translation.z = position[2]
                t.transform.rotation.x = orientation[0]
                t.transform.rotation.y = orientation[1]
                t.transform.rotation.z = orientation[2]
                t.transform.rotation.w = orientation[3]
                self.tf_broadcaster.sendTransform(t)

                # 计算并发布平滑后的TF
                smoothed_pos, smoothed_ori = self.pose_smoother.update(
                    i, position, orientation)

                t_smoothed = TransformStamped()
                t_smoothed.header.stamp = depth_msg.header.stamp
                t_smoothed.header.frame_id = depth_msg.header.frame_id
                t_smoothed.child_frame_id = f'energy_unit_{i}_smoothed'
                t_smoothed.transform.translation.x = smoothed_pos[0]
                t_smoothed.transform.translation.y = smoothed_pos[1]
                t_smoothed.transform.translation.z = smoothed_pos[2]
                t_smoothed.transform.rotation.x = smoothed_ori[0]
                t_smoothed.transform.rotation.y = smoothed_ori[1]
                t_smoothed.transform.rotation.z = smoothed_ori[2]
                t_smoothed.transform.rotation.w = smoothed_ori[3]
                self.tf_broadcaster.sendTransform(t_smoothed)

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
