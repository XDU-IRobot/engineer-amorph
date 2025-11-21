import os
import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import SetBool
from amorph_msgs.srv import GetAccuratePose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import tf2_ros
from ament_index_python.packages import get_package_share_directory
import message_filters

class EnergyUnitLocator(Node):
    def __init__(self):
        super().__init__('energy_unit_locator')

        # 参数
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        
        # 开关服务的状态
        self.enabled = True
        self.srv = self.create_service(SetBool, 'enable_locator', self.enable_callback)

        # 精确测量服务
        self.collecting = False
        self.collected_poses = []
        self.cb_group = ReentrantCallbackGroup()
        self.measure_srv = self.create_service(GetAccuratePose, 'get_accurate_pose', self.measure_callback, callback_group=self.cb_group)

        # 获取模型路径
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            # 默认为已安装的资源
            pkg_share = get_package_share_directory('energy_unit_locator')
            model_path = os.path.join(pkg_share, 'assets', '251120_epoch335.pt')
        
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.bridge = CvBridge()

        # 相机内参
        self.intrinsics = None
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.info_callback,
            10)

        # 对齐到彩色的深度图
        self.sub_rgb = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.sub_depth = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

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

    def measure_callback(self, request, response):
        self.get_logger().info(f"Starting accurate measurement for {request.duration} seconds...")
        
        # 确保节点是开启的
        original_enabled = self.enabled
        self.enabled = True
        
        self.collected_poses = []
        self.collecting = True
        
        # 收集数据
        time.sleep(request.duration)
        
        self.collecting = False
        self.enabled = original_enabled # 恢复原来的状态
        
        if not self.collected_poses:
            response.success = False
            response.message = "No energy units detected during the measurement period."
            self.get_logger().warn(response.message)
            return response
            
        # 数据融合：去除离群点并取平均
        poses = np.array(self.collected_poses) # shape: (N, 3)
        
        # 简单的统计滤波：去除距离均值超过2倍标准差的点
        mean = np.mean(poses, axis=0)
        std = np.std(poses, axis=0)
        
        # 避免标准差为0的情况
        std[std == 0] = 1e-6
        
        z_scores = np.abs((poses - mean) / std)
        valid_mask = (z_scores < 2).all(axis=1)
        
        valid_poses = poses[valid_mask]
        
        if len(valid_poses) == 0:
            # 如果所有点都被过滤了（不太可能），就退回到原始均值
            final_pose = mean
        else:
            final_pose = np.mean(valid_poses, axis=0)
            
        response.success = True
        response.message = f"Measurement successful. Collected {len(poses)} samples, used {len(valid_poses)} valid samples."
        response.pose.position.x = final_pose[0]
        response.pose.position.y = final_pose[1]
        response.pose.position.z = final_pose[2]
        # 旋转保持默认（单位四元数）
        response.pose.orientation.w = 1.0
        
        self.get_logger().info(f"Result: x={final_pose[0]:.4f}, y={final_pose[1]:.4f}, z={final_pose[2]:.4f}")
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

            for i, mask in enumerate(masks):
                binary_mask = mask.astype(bool)
                
                # 出来的mask可能会有一点边边露出背景，所以把边缘腐蚀掉一点，确保只取到目标部分
                kernel = np.ones((5, 5), np.uint8)
                eroded_mask = cv2.erode(mask.astype(np.uint8), kernel, iterations=1)
                binary_mask = eroded_mask.astype(bool)

                cX, cY, z_meters = self.compute_depth_and_centroid(mask, boxes[i], cv_depth, binary_mask)
                
                if z_meters is None:
                    continue

                # 反投影到3D
                # X = (u - cx) * Z / fx
                # Y = (v - cy) * Z / fy
                x_meters = (cX - self.intrinsics['cx']) * z_meters / self.intrinsics['fx']
                y_meters = (cY - self.intrinsics['cy']) * z_meters / self.intrinsics['fy']

                if self.collecting:
                    self.collected_poses.append([x_meters, y_meters, z_meters])

                # 发布TF
                t = TransformStamped()
                t.header.stamp = depth_msg.header.stamp
                t.header.frame_id = depth_msg.header.frame_id # 通常是 camera_link 或 camera_color_optical_frame
                t.child_frame_id = f'energy_unit_{i}'
                
                t.transform.translation.x = x_meters
                t.transform.translation.y = y_meters
                t.transform.translation.z = z_meters
                
                # TODO: 这里没有计算旋转，默认单位四元数，后面可以用mask拟合obb的方法计算朝向
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                self.tf_broadcaster.sendTransform(t)

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
