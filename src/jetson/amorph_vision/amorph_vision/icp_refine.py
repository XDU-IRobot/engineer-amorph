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
        print(f"[Sampler] transform={t1-t0:.4f}s cull={t2-t1:.4f}s total={t2-t0:.4f}s n={result.shape[0]}")
        return result


# ---- 内部辅助 ---------------------------------------------------------




def _huber_weight(r: np.ndarray, k: float) -> np.ndarray:
    """Huber 鲁棒权重: |r|≤k → 1, |r|>k → k/|r|."""
    abs_r = np.abs(r)
    return np.where(abs_r <= k, 1.0, k / abs_r)


# ---- 降自由度 ICP -----------------------------------------------------


class TranslationRefiner:
    """3-DOF: 固定旋转, 只优化 xyz 平移 (点对面 + Huber)."""

    def __init__(self, max_dist: float = 0.05, max_iter: int = 30,
                 huber_k: float = 0.01):
        self.max_dist = max_dist
        self.max_iter = max_iter
        self.huber_k = huber_k

    def refine(self, source_pts: np.ndarray, target_pts: np.ndarray,
               init_pose: dict) -> dict | None:
        if len(source_pts) < 10 or len(target_pts) < 10:
            return None

        t_pca = np.array(init_pose['position'], dtype=np.float64)
        src = source_pts[:, :3].astype(np.float64)

        # 下采样 source 点以加速
        n_src = len(src)
        if n_src > 3000:
            rng = np.random.default_rng(42)
            src = src[rng.choice(n_src, 3000, replace=False)]

        tgt = target_pts[:, :3].astype(np.float64)

        tgt_pcd = o3d.geometry.PointCloud()
        tgt_pcd.points = o3d.utility.Vector3dVector(tgt)

        t_norm = time()
        tgt_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        tgt_normals = np.asarray(tgt_pcd.normals)
        tgt_tree = cKDTree(tgt)
        print(f"[T-ICP] setup: normal+kdtree={time()-t_norm:.4f}s src={len(src)}")

        delta = np.zeros(3)
        prev_rmse, prev_fit = float("inf"), 0.0

        t_loop = time()
        for it in range(self.max_iter):
            src_t = src + delta
            dist, idx = tgt_tree.query(src_t, k=1)
            valid = dist < self.max_dist
            if valid.sum() < 10:
                break

            i_v = idx[valid]                             # target 最近邻索引
            n_v = tgt_normals[i_v]                       # 对应法向量 (K,3)
            tgt_v = tgt[i_v]
            src_v = src_t[valid]

            b = np.sum((tgt_v - src_v) * n_v, axis=1)    # 法向残差 (K,)
            w = _huber_weight(b, self.huber_k)
            d_delta, _, _, _ = np.linalg.lstsq(n_v * w[:, None], b * w, rcond=None)
            delta += d_delta

            fit = valid.sum() / len(src)
            rmse = np.sqrt(np.mean(b ** 2))
            if it > 0 and abs(prev_rmse - rmse) / max(prev_rmse, 1e-10) < 1e-6 and \
               abs(prev_fit - fit) / max(prev_fit, 1e-10) < 1e-6:
                break
            prev_rmse, prev_fit = rmse, fit

        print(f"[T-ICP] delta={delta} n_iter={it+1} loop_time={time()-t_loop:.4f}s fitness={prev_fit:.4f} rmse={prev_rmse:.4f}")
        return {
            'position': (t_pca + delta).tolist(),
            'orientation': init_pose['orientation'],
            'fitness': prev_fit, 'rmse': prev_rmse,
        }


class RayOffsetRefiner:
    """1-DOF: 沿相机光心→PCA 位置射线优化标量偏移 (TranslationRefiner 回退)."""

    def __init__(self, max_dist: float = 0.05, max_iter: int = 50,
                 huber_k: float = 0.01):
        self.max_dist = max_dist
        self.max_iter = max_iter
        self.huber_k = huber_k

    def refine(self, source_pts: np.ndarray, target_pts: np.ndarray,
               init_pose: dict) -> dict | None:
        if len(source_pts) < 10 or len(target_pts) < 10:
            return None

        t_pca = np.array(init_pose['position'], dtype=np.float64)
        r = float(np.linalg.norm(t_pca))
        if r < 1e-6:
            return None
        d = t_pca / r

        src = source_pts[:, :3].astype(np.float64)

        n_src = len(src)
        if n_src > 3000:
            rng = np.random.default_rng(42)
            src = src[rng.choice(n_src, 3000, replace=False)]

        tgt = target_pts[:, :3].astype(np.float64)

        tgt_pcd = o3d.geometry.PointCloud()
        tgt_pcd.points = o3d.utility.Vector3dVector(tgt)

        t_norm = time()
        tgt_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        tgt_normals = np.asarray(tgt_pcd.normals)
        tgt_tree = cKDTree(tgt)
        print(f"[R-ICP] setup: normal+kdtree={time()-t_norm:.4f}s src={len(src)}")

        lam = 0.0
        prev_rmse, prev_fit = float("inf"), 0.0

        t_loop = time()
        for it in range(self.max_iter):
            src_t = src + lam * d
            dist, idx = tgt_tree.query(src_t, k=1)
            valid = dist < self.max_dist
            if valid.sum() < 10:
                break

            i_v = idx[valid]
            n_v = tgt_normals[i_v]
            tgt_v = tgt[i_v]
            src_v = src_t[valid]

            b = np.sum((tgt_v - src_v) * n_v, axis=1)    # 法向残差 (K,)
            a = np.sum(d * n_v, axis=1)                   # 射线方向在法向的投影

            w = _huber_weight(b, self.huber_k)
            num, den = np.sum(w * a * b), np.sum(w * a * a)
            if abs(den) < 1e-12:
                break
            lam += num / den

            fit = valid.sum() / len(src)
            rmse = np.sqrt(np.mean(b ** 2))
            if it > 0 and abs(prev_rmse - rmse) / max(prev_rmse, 1e-10) < 1e-4 and \
               abs(prev_fit - fit) / max(prev_fit, 1e-10) < 1e-4:
                break
            prev_rmse, prev_fit = rmse, fit

        print(f"[R-ICP] λ={lam:.4f} offset={lam*r:.4f}m n_iter={it+1} loop_time={time()-t_loop:.4f}s fitness={prev_fit:.4f} rmse={prev_rmse:.4f}")
        return {
            'position': (t_pca + lam * d).tolist(),
            'orientation': init_pose['orientation'],
            'fitness': prev_fit, 'rmse': prev_rmse,
        }

def refine_pose(source_pts: np.ndarray, target_pts: np.ndarray,
                init_pose: dict, cyl_radius: float = 0.027) -> dict:
    """用降自由度 ICP 精化 PCA 位姿的平移分量.

    策略: 先用 3-DOF 平移 ICP 解 xyz; 若 fitness < 阈值则回退到
    1-DOF 射线 ICP (沿相机→PCA 方向优化标量偏移)。
    两种方法都冻结 PCA 旋转, 只优化平移。

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

    t0 = time()
    # Step 1: 3-DOF 平移 ICP
    t_refiner = TranslationRefiner(max_dist=0.05, max_iter=30, huber_k=0.01)
    result = t_refiner.refine(source_pts, target_pts, init_pose)
    t1 = time()

    if result is not None and result['fitness'] > 0.3:
        print(f"[refine] 3-DOF ok, total={t1-t0:.4f}s")
        return result

    # Step 2: 回退到 1-DOF 射线 ICP
    print("[refine] 3-DOF ICP 不收敛, 回退到 1-DOF 射线 ICP")
    r_refiner = RayOffsetRefiner(max_dist=0.05, max_iter=50, huber_k=0.01)
    result = r_refiner.refine(source_pts, target_pts, init_pose)
    print(f"[refine] 1-DOF fallback, total={time()-t0:.4f}s")

    return result if result is not None else init_pose
