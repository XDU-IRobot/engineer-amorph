import os
# 解决 Jetson 平台 libgomp.so.1 TLS 加载问题
import sys
import ctypes

from sympy import true
try:
    ctypes.CDLL('/lib/aarch64-linux-gnu/libgomp.so.1', mode=ctypes.RTLD_GLOBAL)
except OSError:
    # 非 ARM64 平台或库不存在时忽略
    pass

import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import SetBool
import struct
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import tf2_ros
from ament_index_python.packages import get_package_share_directory
import message_filters

from viztracer import trace_and_save

class EnergyUnitLocator(Node):
    def get_cylinder_radii(self, z_meters):
        """
        根据距离返回上下两端圆柱半径（米）
        近距离: 下端0.095, 上端0.085
        远距离: 两端都为0.09
        """
        if z_meters < 1.1:
            return 0.095, 0.085
        else:
            return 0.09, 0.09
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
        
        # 开关服务的状态
        self.enabled = True
        self.srv = self.create_service(SetBool, 'enable_locator', self.enable_callback)
        
        # 圆柱体的参数
        self.cylinder_height = 0.15  # 米
        self.cylinder_radius_slim = 0.03  # 米
        self.cylinder_radius_fat = 0.09 / 2  # 米

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
            self.get_logger().info(f'Camera Intrinsics received: {self.intrinsics}')

    def compute_depth_and_centroid(self, mask, box, cv_depth, binary_mask):
        # 计算mask的质心
        M = cv2.moments(mask.astype(np.uint8))
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            # 如果矩计算失败，回退到边界框中心
            cX = int((box[0] + box[2]) / 2)
            cY = int((box[1] + box[3]) / 2)

        # 针对"中间细两头粗"的物体，对整个mask取平均会导致深度值被粗的两头拉低（偏近）。
        # 更准确的方法是取质心（通常在物体中间）附近区域的深度中位数。
        
        avg_depth_mm = 0.0
        
        # 1. 尝试采样质心附近区域 (ROI)
        sample_radius = 10  # 采样半径像素
        x_min = max(0, cX - sample_radius)
        x_max = min(cv_depth.shape[1], cX + sample_radius)
        y_min = max(0, cY - sample_radius)
        y_max = min(cv_depth.shape[0], cY + sample_radius)
        
        # 提取ROI内的深度和mask
        roi_depth = cv_depth[y_min:y_max, x_min:x_max]
        roi_mask = binary_mask[y_min:y_max, x_min:x_max]
        
        # 只取 mask 覆盖部分的有效深度
        valid_roi_depth = roi_depth[(roi_mask) & (roi_depth > 0)]
        
        if len(valid_roi_depth) > 5: # 确保有足够的点
            # 使用中位数 (Median) 抗噪，比平均值更稳健
            avg_depth_mm = np.median(valid_roi_depth)
        else:
            # 2. 回退方案：如果质心处没有有效深度（比如正好是黑洞），则使用整个 mask 的中位数
            masked_depth = cv_depth[binary_mask]
            valid_depth = masked_depth[masked_depth > 0]
            
            if len(valid_depth) > 0:
                    avg_depth_mm = np.median(valid_depth)
            else:
                return None, None, None # 无法计算深度

        z_meters = avg_depth_mm / 1000.0
        return cX, cY, z_meters
    
    def estimate_cylinder_orientation(self, binary_mask, cv_depth):
        """
        使用PCA估计圆柱体的主轴方向
        
        参数:
            binary_mask: 二值mask
            cv_depth: 深度图
            
        返回:
            axis_direction: 主轴方向向量 (3,) 在相机坐标系中
            orientation_quat: 四元数 (x, y, z, w)
        """
        # 提取mask区域的所有3D点
        ys, xs = np.where(binary_mask)
        
        if len(xs) < 10:  # 点太少无法计算
            return None, None
        
        depths = cv_depth[ys, xs]
        valid_mask = depths > 0
        ys_valid = ys[valid_mask]
        xs_valid = xs[valid_mask]
        depths_valid = depths[valid_mask]
        if len(depths_valid) < 10:
            return None, None
        z = depths_valid / 1000.0
        px = (xs_valid - self.intrinsics['cx']) * z / self.intrinsics['fx']
        py = (ys_valid - self.intrinsics['cy']) * z / self.intrinsics['fy']
        points_3d = np.column_stack((px, py, z))
        
        # 使用PCA找到主方向
        # 1. 中心化点云
        centroid = np.mean(points_3d, axis=0)
        centered_points = points_3d - centroid
        
        # 2. 计算协方差矩阵
        cov_matrix = np.cov(centered_points.T)
        
        # 3. 特征值分解
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        
        # 4. 主轴是最大特征值对应的特征向量
        sort_indices = np.argsort(eigenvalues)[::-1]
        principal_axis = eigenvectors[:, sort_indices[0]].real
        
        # 归一化主轴方向，避免上下对称导致的方向跳动
        # 策略：强制主轴的y分量为负（或选择其他一致的约束）
        # 这样可以将两个相反的方向统一到同一个方向
        # 根据日志，y分量的绝对值较大，所以用y分量作为判断依据
        if principal_axis[1] > 0:
            principal_axis = -principal_axis
        
        # 将主轴方向转换为四元数
        # 我们假设圆柱体的局部z轴是主轴方向
        # 需要计算从标准z轴 [0, 0, 1] 旋转到 principal_axis 的四元数
        quat = self.rotation_from_vectors(np.array([0, 0, 1]), principal_axis)
        
        return principal_axis, quat
    
    def rotation_from_vectors(self, vec1, vec2):
        """
        计算从vec1旋转到vec2的四元数
        
        参数:
            vec1: 起始向量 (3,)
            vec2: 目标向量 (3,)
            
        返回:
            quaternion: (x, y, z, w)
        """
        # 归一化
        v1 = vec1 / np.linalg.norm(vec1)
        v2 = vec2 / np.linalg.norm(vec2)
        
        # 计算旋转轴和角度
        cross = np.cross(v1, v2)
        dot = np.dot(v1, v2)
        
        # 处理特殊情况：向量平行或反平行
        if np.allclose(cross, 0):
            if dot > 0:  # 平行
                return np.array([0, 0, 0, 1])
            else:  # 反平行，选择一个垂直轴
                # 找一个垂直于v1的向量
                if abs(v1[0]) < 0.9:
                    axis = np.cross(v1, [1, 0, 0])
                else:
                    axis = np.cross(v1, [0, 1, 0])
                axis = axis / np.linalg.norm(axis)
                # 返回180度旋转的四元数
                return np.array([axis[0], axis[1], axis[2], 0])
        
        # 四元数公式
        # q = [cross, 1 + dot] 然后归一化
        quat = np.array([cross[0], cross[1], cross[2], 1 + dot])
        quat = quat / np.linalg.norm(quat)
        
        return quat

    # @trace_and_save
    def image_callback(self, rgb_msg, depth_msg):
        if not self.enabled:
            return

        if self.intrinsics is None:
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
                # 1. 腐蚀去除边缘
                kernel = np.ones((self.erosion_kernel_size, self.erosion_kernel_size), np.uint8)
                eroded_mask = cv2.erode(mask.astype(np.uint8), kernel, iterations=self.erosion_iterations)
                # 2. 连通域筛选，只保留最大连通域
                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(eroded_mask, connectivity=8)
                if num_labels <= 1:
                    continue
                largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
                filtered_mask = (labels == largest_label)
                # 3. 深度约束，去除与质心深度差异较大的点
                ys, xs = np.where(filtered_mask)
                if len(xs) < 10:
                    continue
                depths = cv_depth[ys, xs]
                valid = depths > 0
                if np.sum(valid) < 5:
                    continue
                median_depth = np.median(depths[valid])
                depth_thresh = 0.08  # 8cm阈值，可根据实际调整
                # 用NumPy一次性处理深度约束
                depth_valid = depths > 0
                depth_diff = np.abs(depths/1000.0 - median_depth/1000.0)
                mask_valid = (depth_valid) & (depth_diff < depth_thresh)
                depth_mask = np.zeros_like(filtered_mask, dtype=bool)
                depth_mask[ys[mask_valid], xs[mask_valid]] = True
                binary_mask = depth_mask

                cX, cY, z_meters = self.compute_depth_and_centroid(mask, boxes[i], cv_depth, binary_mask)
                if z_meters is None:
                    continue

                # 根据距离动态选择圆柱两端半径
                radius_bottom, radius_top = self.get_cylinder_radii(z_meters)
                # 可在后续拟合/点云处理等环节使用 radius_bottom, radius_top

                # 反投影到3D
                x_meters = (cX - self.intrinsics['cx']) * z_meters / self.intrinsics['fx']
                y_meters = (cY - self.intrinsics['cy']) * z_meters / self.intrinsics['fy']

                # 估计圆柱体主轴方向
                axis_direction, orientation_quat = self.estimate_cylinder_orientation(binary_mask, cv_depth)
                if orientation_quat is None:
                    orientation_quat = np.array([0.0, 0.0, 0.0, 1.0])
                    self.get_logger().warn(f'Failed to estimate orientation for energy_unit_{i}, using default')


                # 优化点云生成：用NumPy批量处理
                ys, xs = np.where(binary_mask)
                depths = cv_depth[ys, xs]
                valid_mask = depths > 0
                ys = ys[valid_mask]
                xs = xs[valid_mask]
                depths = depths[valid_mask]
                z = depths / 1000.0
                px = (xs - self.intrinsics['cx']) * z / self.intrinsics['fx']
                py = (ys - self.intrinsics['cy']) * z / self.intrinsics['fy']
                rgbs = cv_rgb[ys, xs]
                # 颜色打包为uint32
                rgb_uint32 = np.array([struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0] for b, g, r in rgbs], dtype=np.uint32)
                # 拼成点云数组
                if len(px) > 0:
                    all_points_np = np.column_stack((px, py, z, rgb_uint32))
                    all_points.extend(all_points_np.tolist())

                # 沿camera_link到圆柱表面方向推进3cm，使TF位于圆柱中心
                # camera_link在(0,0,0)，圆柱表面TF在(x_meters, y_meters, z_meters)
                direction = np.array([x_meters, y_meters, z_meters])
                norm = np.linalg.norm(direction)
                if norm > 1e-6:
                    offset = 0.03 * direction / norm
                    x_tf = x_meters + offset[0]
                    y_tf = y_meters + offset[1]
                    z_tf = z_meters + offset[2]
                else:
                    x_tf, y_tf, z_tf = x_meters, y_meters, z_meters

                t = TransformStamped()
                t.header.stamp = depth_msg.header.stamp
                t.header.frame_id = depth_msg.header.frame_id
                t.child_frame_id = f'energy_unit_{i}'
                t.transform.translation.x = x_tf
                t.transform.translation.y = y_tf
                t.transform.translation.z = z_tf
                t.transform.rotation.x = float(orientation_quat[0])
                t.transform.rotation.y = float(orientation_quat[1])
                t.transform.rotation.z = float(orientation_quat[2])
                t.transform.rotation.w = float(orientation_quat[3])
                self.tf_broadcaster.sendTransform(t)
                if axis_direction is not None:
                    self.get_logger().info(
                        f'Energy unit {i}: pos=({x_meters:.3f}, {y_meters:.3f}, {z_meters:.3f}), '
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
