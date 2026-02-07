# 解决 Jetson 平台 libgomp.so.1 TLS 加载问题
import ctypes
try:
    ctypes.CDLL('/lib/aarch64-linux-gnu/libgomp.so.1', mode=ctypes.RTLD_GLOBAL)
except OSError:
    # 非 ARM64 平台或库不存在时忽略
    pass

# std
import os
import struct

# thirdparty
import cv2
import numpy as np
from ultralytics import YOLO  # type: ignore

# ros
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
import message_filters
import tf2_ros
from ament_index_python.packages import get_package_share_directory

# ros interfaces
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import SetBool

# local
from .pose_smoother import PoseSmoother
from .locator_backend import LocatorBackend

# profiler
# from viztracer import trace_and_save

class EnergyUnitLocator(Node):
    def __init__(self):
        super().__init__('energy_unit_locator')

        # 参数
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('erosion_kernel_size', 5)
        self.declare_parameter('erosion_iterations', 1)
        # 点云发布开关参数
        self.declare_parameter('publish_masked_pointcloud', False)
        self.publish_masked_pointcloud = self.get_parameter('publish_masked_pointcloud').get_parameter_value().bool_value
        
        # 平滑滤波参数
        self.declare_parameter('smoothing_alpha', 0.3)  # 平滑因子，越小越平滑
        smoothing_alpha = self.get_parameter('smoothing_alpha').get_parameter_value().double_value
        
        # 位姿平滑器
        self.pose_smoother = PoseSmoother(alpha=smoothing_alpha)
        
        # 定位后端（在收到相机内参后初始化）
        self.locator_backend = None
        
        # 开关服务的状态
        self.enabled = True
        self.srv = self.create_service(SetBool, 'enable_locator', self.enable_callback)

        # 获取模型路径
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            # 默认为已安装的资源
            pkg_share = get_package_share_directory('energy_unit_locator')
            model_path = os.path.join(pkg_share, 'assets', '251120_epoch335.pt')
        
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.erosion_kernel_size = self.get_parameter('erosion_kernel_size').get_parameter_value().integer_value
        self.erosion_iterations = self.get_parameter('erosion_iterations').get_parameter_value().integer_value

        # tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 点云发布器,用于可视化调试
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'masked_pointcloud', 10)

        self.bridge = CvBridge()

        # 相机内参
        self.intrinsics = None
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.info_callback,
            10)

        # 对齐到彩色的深度图
        self.sub_rgb = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.sub_depth = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.image_callback)

        self.get_logger().info('Energy Unit Locator Node Started')

    def enable_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f"Locator enabled: {self.enabled}"
        self.get_logger().info(response.message)
        return response


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
            self.get_logger().info(f'Camera Intrinsics received: {self.intrinsics}')

    # @trace_and_save
    def image_callback(self, rgb_msg, depth_msg):
        if not self.enabled:
            return

        if self.locator_backend is None:
            self.get_logger().warn('Waiting for camera intrinsics...', throttle_duration_sec=2.0)
            return

        try:
            # 将 ROS 图像转换为 OpenCV 格式
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough') # 通常是 16UC1
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # 推理
        results = self.model(cv_rgb, verbose=False, conf=self.conf_thresh)

        for result in results:
            if result.masks is None:
                continue

            masks = result.masks.data.cpu().numpy()
            boxes = result.boxes.data.cpu().numpy()
            
            # 推理出来的mask大小可能和深度图不匹配，调整一下
            if masks.shape[1:] != cv_depth.shape:
                 masks = np.array([cv2.resize(mask, (cv_depth.shape[1], cv_depth.shape[0]), interpolation=cv2.INTER_NEAREST) for mask in masks])

            # 收集所有mask后的点云用于可视化
            all_points = []
            
            for i, mask in enumerate(masks):
                # 使用后端处理mask
                pose_result = self.locator_backend.process_mask(mask, boxes[i], cv_depth)
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
                
                # 生成点云用于可视化
                if self.publish_masked_pointcloud:
                    ys, xs = np.where(binary_mask)
                    depths = cv_depth[ys, xs]
                    valid_mask = depths > 0
                    ys = ys[valid_mask]
                    xs = xs[valid_mask]
                    depths = depths[valid_mask]
                    z = depths / 1000.0
                    px = (xs - self.intrinsics['cx']) * z / self.intrinsics['fx'] # type: ignore
                    py = (ys - self.intrinsics['cy']) * z / self.intrinsics['fy'] # type: ignore
                    rgbs = cv_rgb[ys, xs]
                    # 颜色打包为uint32
                    rgb_uint32 = np.array([struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0] for b, g, r in rgbs], dtype=np.uint32)
                    # 拼成点云数组
                    if len(px) > 0:
                        all_points_np = np.column_stack((px, py, z, rgb_uint32))
                        all_points.extend(all_points_np.tolist())
                
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
                smoothed_pos, smoothed_ori = self.pose_smoother.update(i, position, orientation)
                
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
                
                # 日志输出
                if axis_direction is not None:
                    self.get_logger().info(
                        f'Energy unit {i}: pos=({raw_position[0]:.3f}, {raw_position[1]:.3f}, {raw_position[2]:.3f}), '
                        f'axis=({axis_direction[0]:.3f}, {axis_direction[1]:.3f}, {axis_direction[2]:.3f}), '
                        f'radius_bottom={radius_bottom:.3f}, radius_top={radius_top:.3f}'
                    )
            
            # 发布点云（可开关）
            if self.publish_masked_pointcloud and all_points:
                self.publish_pointcloud(all_points, depth_msg.header)
    
    def publish_pointcloud(self, points, header):
        """发布点云用于可视化"""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        points_np = np.array(points)
        # 确保shape为(N,4)，前三列float32，最后一列uint32
        if points_np.shape[1] == 4:
            xyz = points_np[:, :3].astype(np.float32)
            rgb = points_np[:, 3].astype(np.uint32)
            structured = np.zeros(points_np.shape[0], dtype={'names': ['x', 'y', 'z', 'rgb'], 'formats': ['<f4', '<f4', '<f4', '<u4']})
            structured['x'] = xyz[:, 0]
            structured['y'] = xyz[:, 1]
            structured['z'] = xyz[:, 2]
            structured['rgb'] = rgb
            cloud_data = structured.tobytes()
        else:
            cloud_data = b''
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = cloud_data
        self.pointcloud_pub.publish(cloud_msg)

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
