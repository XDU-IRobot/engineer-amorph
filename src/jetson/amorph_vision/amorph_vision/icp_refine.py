"""ICP 精配准: PLY 模型可见表面采样 + 降自由度点对面 ICP."""

import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from time import time


def extract_masked_points(depth: np.ndarray, color: np.ndarray, mask: np.ndarray,
                          intrinsics: dict) -> np.ndarray:
    """从深度图提取 mask 区域的 3D 点云 (相机系, 米), N×6 (x,y,z,r,g,b)."""
    valid = (depth > 0) & mask
    ys, xs = np.where(valid)
    if len(xs) == 0:
        return np.empty((0, 6), dtype=np.float32)

    z = depth[ys, xs] / 1000.0
    x = (xs - intrinsics['cx']) * z / intrinsics['fx']
    y = (ys - intrinsics['cy']) * z / intrinsics['fy']
    rgb = color[ys, xs][:, ::-1].astype(np.float32)  # BGR->RGB
    return np.column_stack([x, y, z, rgb]).astype(np.float32)


class SurfaceCullingSampler:
    """从 PLY 模型采样面向相机的可见表面点云.

    加载 mesh -> 采样 50000 点 -> 归中.
    采样时按位姿放置模型, 以相机原点为球心做半径裁剪剔除背面.
    """

    def __init__(self, ply_path: str, intrinsics: dict):
        self.intrinsics = intrinsics
        mesh = o3d.io.read_triangle_mesh(ply_path)
        pcd = mesh.sample_points_poisson_disk(
            number_of_points=50000, init_factor=5)
        self._vertices = np.asarray(pcd.points).astype(np.float64)
        self._centroid = self._vertices.mean(axis=0)
        self._vertices -= self._centroid               # 归中到原点
        self._max_extent = float(np.linalg.norm(self._vertices, axis=1).max())
        if pcd.has_colors():
            self._colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
        else:
            self._colors = np.full_like(self._vertices, 200, dtype=np.uint8)
        print(
            f"Loaded {len(self._vertices)} model points, max_extent={self._max_extent:.4f}m")

    def sample_visible(self, pose: dict, clip_factor: float = 0.975,
                       to_camera: bool = True) -> np.ndarray:
        """按位姿放置模型并球面裁剪, 返回 N×6 (x,y,z,r,g,b)."""
        t0 = time()
        R = Rotation.from_quat(pose['orientation']).as_matrix()
        t = np.array(pose['position'], dtype=np.float64)

        # 将归中后的模型点变换到位姿处
        if to_camera:
            pts = self._vertices @ R.T + t
        else:
            pts = self._vertices.copy()
        pts_cam = pts if to_camera else self._vertices @ R.T + t
        t1 = time()

        # 球面裁剪: 以相机原点为球心, 距离 > clip_radius 的点视为背面/不可见
        clip_radius = clip_factor * float(np.linalg.norm(t))
        keep = np.linalg.norm(pts_cam, axis=1) <= clip_radius
        result = np.column_stack([pts[keep], self._colors[keep]])
        t2 = time()
        # print(f"[Sampler] transform={t1-t0:.4f}s cull={t2-t1:.4f}s total={t2-t0:.4f}s n={result.shape[0]}")
        return result


# ---- 内部辅助 ---------------------------------------------------------

_EPS = 1e-12


def _huber_weight(r: np.ndarray, k: float) -> np.ndarray:
    """Huber 鲁棒权重: |r|≤k → 1, |r|>k → k/|r|."""
    abs_r = np.abs(r)
    return np.where(abs_r <= k, 1.0, k / abs_r)


def _subsample(points: np.ndarray, max_points: int) -> np.ndarray:
    """等间隔抽稀到 max_points 个点.

    等间隔而非随机采样: 结果与帧序无关, 不会给位姿引入随机抖动.
    """
    if max_points <= 0 or len(points) <= max_points:
        return points
    idx = np.linspace(0, len(points) - 1, max_points).astype(np.int64)
    return points[idx]


def _prepare_target(target_pts: np.ndarray, voxel_size: float
                    ) -> tuple[np.ndarray, np.ndarray]:
    """体素降采样目标点云并估计法向, 供整个迭代过程复用.

    降采样让最近邻查询和法向估计都明显变快; 用 Open3D 的 C++ 实现.
    """
    pcd = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(target_pts[:, :3].astype(np.float64)))
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=16))
    return np.asarray(pcd.points), np.asarray(pcd.normals)


def _solve_translation(normals: np.ndarray, residual: np.ndarray,
                       weights: np.ndarray) -> np.ndarray:
    """加权点对面残差的 3x3 法方程解 (比 lstsq 的 SVD 快两个数量级)."""
    a = normals * weights[:, None]
    return np.linalg.solve(a.T @ a + _EPS * np.eye(3), a.T @ (residual * weights))


# ---- 降自由度 ICP -----------------------------------------------------


class TranslationRefiner:
    """3-DOF: 固定旋转, 只优化 xyz 平移 (点对面 + Huber).

    旋转被冻结在 PCA 结果上, 因此不能用 Open3D 的 registration_icp
    (那是 6-DOF, 会把旋转误差折算进平移分量). 这里只借它的
    voxel_down_sample / estimate_normals 两个 C++ 组件, 迭代本身用
    numpy 向量化, 并靠步长早停把迭代次数从 30 降到 5~8 次.
    """

    def __init__(self, max_dist: float = 0.05, max_iter: int = 15,
                 huber_k: float = 0.01, max_points: int = 800,
                 voxel_size: float = 0.004, tol: float = 1e-5):
        self.max_dist = max_dist
        self.max_iter = max_iter
        self.huber_k = huber_k
        self.max_points = max_points
        self.voxel_size = voxel_size
        self.tol = tol

    def refine(self, source_pts: np.ndarray, target_pts: np.ndarray,
               init_pose: dict) -> dict | None:
        if len(source_pts) < 10 or len(target_pts) < 10:
            return None

        t_pca = np.array(init_pose['position'], dtype=np.float64)
        src = _subsample(source_pts[:, :3].astype(np.float64), self.max_points)

        tgt, tgt_normals = _prepare_target(target_pts, self.voxel_size)
        if len(tgt) < 10:
            return None
        tgt_tree = cKDTree(tgt)

        delta = np.zeros(3)
        prev_rmse, prev_fit = float("inf"), 0.0
        for _ in range(self.max_iter):
            dist, idx = tgt_tree.query(src + delta, k=1)
            valid = dist < self.max_dist
            if valid.sum() < 10:
                break

            i_v = idx[valid]                             # target 最近邻索引
            n_v = tgt_normals[i_v]                       # 对应法向量 (K,3)
            residual = np.sum((tgt[i_v] - (src + delta)[valid]) * n_v, axis=1)

            w = _huber_weight(residual, self.huber_k)
            step = _solve_translation(n_v, residual, w)
            delta += step

            prev_fit = valid.sum() / len(src)
            prev_rmse = float(np.sqrt(np.mean(residual ** 2)))
            if np.linalg.norm(step) < self.tol:
                break

        return {
            'position': (t_pca + delta).tolist(),
            'orientation': init_pose['orientation'],
            'fitness': prev_fit, 'rmse': prev_rmse,
        }


class RayOffsetRefiner:
    """1-DOF: 沿相机光心→PCA 位置射线优化标量偏移 (TranslationRefiner 回退).

    与 TranslationRefiner 共用降采样/法向预处理与步长早停.
    """

    def __init__(self, max_dist: float = 0.05, max_iter: int = 15,
                 huber_k: float = 0.01, max_points: int = 800,
                 voxel_size: float = 0.004, tol: float = 1e-5):
        self.max_dist = max_dist
        self.max_iter = max_iter
        self.huber_k = huber_k
        self.max_points = max_points
        self.voxel_size = voxel_size
        self.tol = tol

    def refine(self, source_pts: np.ndarray, target_pts: np.ndarray,
               init_pose: dict) -> dict | None:
        if len(source_pts) < 10 or len(target_pts) < 10:
            return None

        t_pca = np.array(init_pose['position'], dtype=np.float64)
        r = float(np.linalg.norm(t_pca))
        if r < 1e-6:
            return None
        d = t_pca / r

        src = _subsample(source_pts[:, :3].astype(np.float64), self.max_points)

        tgt, tgt_normals = _prepare_target(target_pts, self.voxel_size)
        if len(tgt) < 10:
            return None
        tgt_tree = cKDTree(tgt)

        lam = 0.0
        prev_rmse, prev_fit = float("inf"), 0.0
        for _ in range(self.max_iter):
            dist, idx = tgt_tree.query(src + lam * d, k=1)
            valid = dist < self.max_dist
            if valid.sum() < 10:
                break

            i_v = idx[valid]
            n_v = tgt_normals[i_v]
            residual = np.sum((tgt[i_v] - (src + lam * d)[valid]) * n_v, axis=1)
            projection = np.sum(d * n_v, axis=1)          # 射线方向在法向的投影

            w = _huber_weight(residual, self.huber_k)
            den = np.sum(w * projection * projection)
            if abs(den) < 1e-12:
                break
            step = np.sum(w * projection * residual) / den
            lam += step

            prev_fit = valid.sum() / len(src)
            prev_rmse = float(np.sqrt(np.mean(residual ** 2)))
            if abs(step) * r < self.tol:
                break

        return {
            'position': (t_pca + lam * d).tolist(),
            'orientation': init_pose['orientation'],
            'fitness': prev_fit, 'rmse': prev_rmse,
        }


def refine_pose(source_pts: np.ndarray, target_pts: np.ndarray,
                init_pose: dict, cyl_radius: float = 0.027) -> dict:
    """用降自由度 ICP 精化 PCA 位姿的平移分量.

    策略: 先用 3-DOF 平移 ICP 解 xyz; 若 fitness 不足则回退到
    1-DOF 射线 ICP (沿相机→PCA 方向优化标量偏移)。
    两种方法都冻结 PCA 旋转, 只优化平移。旋转必须锁死: 能量单元在
    可见面上接近对称, 放开旋转后平移分量会去补偿旋转误差。

    Parameters
    ----------
    source_pts : N×6 (x,y,z,r,g,b), 模型采样点, 相机系, 米.
    target_pts : M×6 (x,y,z,r,g,b), 场景点, 相机系, 米.
    init_pose : PCA 粗位姿 {'position', 'orientation'}.
    cyl_radius : 预留, 暂未使用.

    Returns
    -------
    dict: 精化位姿; 若均失败则回退 init_pose.
    """
    if len(source_pts) < 50 or len(target_pts) < 50:
        return init_pose

    result = TranslationRefiner().refine(source_pts, target_pts, init_pose)
    if result is not None and result['fitness'] > 0.3:
        return result

    # 3-DOF 不收敛时沿视线方向做 1-DOF 回退
    result = RayOffsetRefiner().refine(source_pts, target_pts, init_pose)
    return result if result is not None else init_pose
