"""单物体位姿估计: PCA粗定位 → ICP精配准."""

import cv2
import numpy as np

from .icp_refine import SurfaceCullingSampler, extract_masked_points, refine_pose
from .pca_localization import PCALocator


class PoseEstimator:
    """封装 PCA→ICP 完整流程, 输入一帧 + 一个检测, 输出位姿."""

    def __init__(self, ply_path: str, intrinsics: dict):
        self.locator = PCALocator(intrinsics)
        self.sampler = SurfaceCullingSampler(ply_path, intrinsics)
        self.intrinsics = intrinsics

    def estimate(self, color: np.ndarray, depth: np.ndarray,
                 raw_mask: np.ndarray, box: list[float]) -> dict | None:
        """估计单物体位姿.

        Parameters
        ----------
        color : (H,W,3) uint8 BGR.
        depth : (H,W) float32, 单位毫米.
        raw_mask : (H',W'), YOLO 输出的原始 mask.
        box : [x1, y1, x2, y2, conf, cls].

        Returns
        -------
        dict: {'position': [x,y,z], 'orientation': [x,y,z,w], ...} 或 None.
        """
        # mask 尺寸对齐到 depth
        h, w = depth.shape[:2]
        mask = raw_mask.astype(np.uint8)
        if mask.shape[:2] != (h, w):
            mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
        mask = mask.astype(bool)

        # PCA 粗定位
        pose = self.locator.process_mask(mask, np.array(box), depth)
        if pose is None:
            return None

        # ICP 精配准
        target_pts = extract_masked_points(
            depth, color, pose['binary_mask'], self.intrinsics)
        source_pts = self.sampler.sample_visible(pose, to_camera=True)

        refined = refine_pose(source_pts, target_pts, pose, cyl_radius=0.027)
        return refined if refined is not None else pose
