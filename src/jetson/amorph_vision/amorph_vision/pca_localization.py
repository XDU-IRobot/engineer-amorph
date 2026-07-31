"""
第一阶段 PCA粗定位 从YOLO给出的RGBD mask大致（masks）计算目标的3D位姿
"""

from typing import Tuple, Optional, Dict, Any
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


class PCALocator:
    """
    定位后端，负责从分割mask计算3D位姿
    """
    
    def __init__(self, 
                 intrinsics: Dict[str, float],
                 erosion_kernel_size: int = 5,
                 erosion_iterations: int = 1):
        """
        初始化定位后端
        
        参数:
            intrinsics: 相机内参字典 {'fx', 'fy', 'cx', 'cy'}
            erosion_kernel_size: 腐蚀核大小
            erosion_iterations: 腐蚀迭代次数
        """
        self.intrinsics = intrinsics
        self.erosion_kernel_size = erosion_kernel_size
        self.erosion_iterations = erosion_iterations
    
    def process_mask(self, 
                    mask: np.ndarray, 
                    box: np.ndarray,
                    depth_image: np.ndarray) -> Optional[Dict[str, Any]]:
        """
        处理单个mask，计算3D位姿
        
        参数:
            mask: 分割mask (H, W)
            box: 边界框 [x1, y1, x2, y2, conf, cls]
            depth_image: 深度图 (H, W)，单位：毫米
            
        返回:
            包含位姿信息的字典，如果处理失败则返回None
            {
                'position': [x, y, z],  # PCA 修正后的3D中心（米）
                'orientation': [x, y, z, w],  # 四元数
                'axis_direction': [x, y, z],  # 主轴方向，或 None
                'binary_mask': np.ndarray,  # 处理后的二值mask
            }
        """
        # 1. 腐蚀去除边缘
        kernel = np.ones((self.erosion_kernel_size, self.erosion_kernel_size), np.uint8)
        eroded_mask = cv2.erode(mask.astype(np.uint8), kernel, iterations=self.erosion_iterations)
        
        # 2. 连通域筛选，只保留最大连通域
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(eroded_mask, connectivity=8)
        if num_labels <= 1:
            print("No connected components found after erosion.")
            return None
        
        largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        filtered_mask = (labels == largest_label)
        
        # 3. 深度约束，去除与质心深度差异较大的点
        ys, xs = np.where(filtered_mask)
        if len(xs) < 10:
            print("Not enough points in the filtered mask for depth processing.")
            return None
        
        depths = depth_image[ys, xs]
        valid = depths > 0
        if np.sum(valid) < 5:
            print("Not enough valid depth points in the filtered mask.")
            return None
        
        median_depth = np.median(depths[valid])
        depth_thresh = 0.08  # 8cm阈值
        
        # 用NumPy一次性处理深度约束
        depth_valid = depths > 0
        depth_diff = np.abs(depths/1000.0 - median_depth/1000.0)
        mask_valid = (depth_valid) & (depth_diff < depth_thresh)
        depth_mask = np.zeros_like(filtered_mask, dtype=bool)
        depth_mask[ys[mask_valid], xs[mask_valid]] = True
        binary_mask = depth_mask
        
        # 4. 计算深度和质心
        centroid_result = self._compute_depth_and_centroid(mask, box, depth_image, binary_mask)
        if centroid_result is None:
            print("Failed to compute depth and centroid.")
            return None
        
        cX, cY, z_meters = centroid_result

        # 5. 反投影到3D
        x_meters = (cX - self.intrinsics['cx']) * z_meters / self.intrinsics['fx']
        y_meters = (cY - self.intrinsics['cy']) * z_meters / self.intrinsics['fy']

        # 6. PCA 估计主轴方向和修正后的3D中心
        axis_direction, orientation_quat, corrected_center = self._estimate_cylinder_orientation(binary_mask, depth_image)
        if orientation_quat is None:
            orientation_quat = np.array([0.0, 0.0, 0.0, 1.0])
            axis_direction = None

        # 使用 PCA 轴向修正后的3D中心
        if corrected_center is not None:
            x_meters, y_meters, z_meters = corrected_center

        return {
            'position': [x_meters, y_meters, z_meters],
            'orientation': orientation_quat.tolist(),
            'axis_direction': axis_direction.tolist() if axis_direction is not None else None,
            'binary_mask': binary_mask,
        }
    
    def _compute_depth_and_centroid(self, 
                                   mask: np.ndarray, 
                                   box: np.ndarray, 
                                   depth_image: np.ndarray, 
                                   binary_mask: np.ndarray) -> Optional[Tuple[int, int, float]]:
        """
        计算mask的质心和深度
        
        返回:
            (cX, cY, z_meters) 或 None
        """
        # 计算mask的质心
        M = cv2.moments(mask.astype(np.uint8))
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            # 如果矩计算失败，回退到边界框中心
            cX = int((box[0] + box[2]) / 2)
            cY = int((box[1] + box[3]) / 2)
        
        # 针对"中间细两头粗"的物体，对整个mask取平均会导致深度值被粗的两头拉低（偏近）
        # 更准确的方法是取质心（通常在物体中间）附近区域的深度中位数
        
        # 1. 尝试采样质心附近区域 (ROI)
        sample_radius = 10  # 采样半径像素
        x_min = max(0, cX - sample_radius)
        x_max = min(depth_image.shape[1], cX + sample_radius)
        y_min = max(0, cY - sample_radius)
        y_max = min(depth_image.shape[0], cY + sample_radius)
        
        # 提取ROI内的深度和mask
        roi_depth = depth_image[y_min:y_max, x_min:x_max]
        roi_mask = binary_mask[y_min:y_max, x_min:x_max]
        
        # 只取 mask 覆盖部分的有效深度
        valid_roi_depth = roi_depth[(roi_mask) & (roi_depth > 0)]
        
        if len(valid_roi_depth) > 5:  # 确保有足够的点
            # 使用中位数 (Median) 抗噪，比平均值更稳健
            avg_depth_mm = np.median(valid_roi_depth)
        else:
            # 2. 回退方案：如果质心处没有有效深度（比如正好是黑洞），则使用整个 mask 的中位数
            masked_depth = depth_image[binary_mask]
            valid_depth = masked_depth[masked_depth > 0]
            
            if len(valid_depth) > 0:
                avg_depth_mm = np.median(valid_depth)
            else:
                return None  # 无法计算深度
        
        z_meters = float(avg_depth_mm / 1000.0)
        return cX, cY, z_meters
    
    def _estimate_cylinder_orientation(self, 
                                      binary_mask: np.ndarray, 
                                      depth_image: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        使用PCA估计圆柱体的主轴方向和轴向修正后的几何中心
        
        参数:
            binary_mask: 二值mask
            depth_image: 深度图
            
        返回:
            (axis_direction, orientation_quat, corrected_center)
            axis_direction: 主轴方向向量 (3,) 在相机坐标系中，或None
            orientation_quat: 四元数 (x, y, z, w)，或None
            corrected_center: 轴向修正后的3D几何中心 (3,)，或None
        """
        # 提取mask区域的所有3D点
        ys, xs = np.where(binary_mask)
        
        if len(xs) < 10:  # 点太少无法计算
            return None, None, None
        
        depths = depth_image[ys, xs]
        valid_mask = depths > 0
        ys_valid = ys[valid_mask]
        xs_valid = xs[valid_mask]
        depths_valid = depths[valid_mask]
        
        if len(depths_valid) < 10:
            return None, None, None
        
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
        if principal_axis[1] > 0:
            principal_axis = -principal_axis
        
        # 5. 计算轴向修正后的几何中心
        # 将所有点投影到主轴上，用端点中点修正轴向位置
        # 这样可以消除透视导致的轴向偏移（仰视/俯视时近端点在图像中占比更大）
        projections = centered_points @ principal_axis  # 每个点在轴上的投影距离
        # 使用2%和98%分位数代替min/max，避免离群点影响
        t_low = np.percentile(projections, 2)
        t_high = np.percentile(projections, 98)
        axis_midpoint_offset = (t_low + t_high) / 2.0
        corrected_center = centroid + axis_midpoint_offset * principal_axis
        
        # 将主轴方向转换为四元数
        # 我们假设圆柱体的局部z轴是主轴方向
        # 需要计算从标准z轴 [0, 0, 1] 旋转到 principal_axis 的四元数
        quat = self._rotation_from_vectors(np.array([0, 0, 1]), principal_axis)
        
        return principal_axis, quat, corrected_center
    
    def _rotation_from_vectors(self, vec1: np.ndarray, vec2: np.ndarray) -> np.ndarray:
        """
        计算从vec1旋转到vec2的四元数
        使用scipy的Rotation.align_vectors方法
        
        参数:
            vec1: 起始向量 (3,)
            vec2: 目标向量 (3,)
            
        返回:
            quaternion: (x, y, z, w)
        """
        # 使用align_vectors: 找到将vec1对齐到vec2的旋转
        # 输入需要是(N, 3)形状
        r, _ = Rotation.align_vectors([vec2], [vec1])
        return r.as_quat()  # 返回 [x, y, z, w]
    
    def update_intrinsics(self, intrinsics: Dict[str, float]):
        """
        更新相机内参
        
        参数:
            intrinsics: 相机内参字典 {'fx', 'fy', 'cx', 'cy'}
        """
        self.intrinsics = intrinsics
