"""
3D SORT tracker
使用卡尔曼滤波 + 匈牙利匹配，为多目标检测分配稳定 track_id。
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass
class TrackAssignment:
    """跟踪输出：某个 track_id 关联到当前帧的检测索引。"""

    track_id: int
    detection_index: int


class _KalmanTrack3D:
    """单个 3D 目标轨迹，状态为 [x, y, z, vx, vy, vz]。"""

    def __init__(
        self,
        track_id: int,
        measurement_xyz: np.ndarray,
        bbox_xyxy: np.ndarray,
        measurement_var: float,
    ) -> None:
        self.track_id = track_id

        self.x = np.zeros(6, dtype=np.float64)
        self.x[:3] = measurement_xyz

        self.P = np.diag([0.05, 0.05, 0.05, 1.0, 1.0, 1.0]).astype(np.float64)
        self.last_bbox = bbox_xyxy.astype(np.float64)

        self.hits = 1
        self.age = 1
        self.missed = 0
        self._measurement_var = measurement_var

    @staticmethod
    def _build_F(dt: float) -> np.ndarray:
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        return F

    @staticmethod
    def _build_Q(dt: float, process_var: float) -> np.ndarray:
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2

        # 常速度模型下，白噪声加速度离散化协方差
        q_block = np.array([[dt4 / 4.0, dt3 / 2.0], [dt3 / 2.0, dt2]], dtype=np.float64)
        Q = np.zeros((6, 6), dtype=np.float64)
        Q[np.ix_([0, 3], [0, 3])] = q_block * process_var
        Q[np.ix_([1, 4], [1, 4])] = q_block * process_var
        Q[np.ix_([2, 5], [2, 5])] = q_block * process_var
        return Q

    @staticmethod
    def _H() -> np.ndarray:
        H = np.zeros((3, 6), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        return H

    def predict(self, dt: float, process_var: float) -> None:
        F = self._build_F(dt)
        Q = self._build_Q(dt, process_var)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.age += 1
        self.missed += 1

    def update(self, measurement_xyz: np.ndarray, bbox_xyxy: np.ndarray) -> None:
        H = self._H()
        R = np.eye(3, dtype=np.float64) * self._measurement_var

        innovation = measurement_xyz - (H @ self.x)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.pinv(S)

        self.x = self.x + K @ innovation
        I = np.eye(6, dtype=np.float64)
        self.P = (I - K @ H) @ self.P

        self.last_bbox = bbox_xyxy.astype(np.float64)
        self.hits += 1
        self.missed = 0

    def mahalanobis_squared(self, measurement_xyz: np.ndarray) -> float:
        H = self._H()
        R = np.eye(3, dtype=np.float64) * self._measurement_var

        innovation = measurement_xyz - (H @ self.x)
        S = H @ self.P @ H.T + R
        d2 = innovation.T @ np.linalg.pinv(S) @ innovation
        return float(d2)


class Sort3DTracker:
    """
    3D SORT 跟踪器。

    检测输入格式:
        {
            'position': [x, y, z],
            'box': [x1, y1, x2, y2],
            ...  # 其他字段原样透传给调用方
        }
    """

    def __init__(
        self,
        max_age: int = 8,
        min_hits: int = 3,
        mahalanobis_thresh: float = 9.0,
        iou_weight: float = 0.2,
        process_var: float = 0.5,
        measurement_var: float = 0.0025,
    ) -> None:
        self.max_age = max_age
        self.min_hits = min_hits
        self.mahalanobis_thresh = mahalanobis_thresh
        self.iou_weight = iou_weight
        self.process_var = process_var
        self.measurement_var = measurement_var

        # LRU ID pool: 0-15, 最多同时跟踪 16 个目标
        self._max_tracks = 16
        self._available_ids: List[int] = list(range(self._max_tracks))
        self._lru_order: List[int] = []  # 最近使用顺序: 队首 = LRU, 队尾 = MRU
        self._tracks: Dict[int, _KalmanTrack3D] = {}

    def reset(self) -> None:
        self._tracks.clear()
        self._available_ids = list(range(self._max_tracks))
        self._lru_order.clear()

    def update(
        self,
        detections: Sequence[Dict[str, Any]],
        dt: float = 1.0,
    ) -> Tuple[List[TrackAssignment], List[int]]:
        dt = max(1e-3, float(dt))

        track_ids = list(self._tracks.keys())
        for track_id in track_ids:
            self._tracks[track_id].predict(dt=dt, process_var=self.process_var)

        if len(detections) == 0:
            removed_ids = self._prune_dead_tracks()
            return [], removed_ids

        det_positions = [np.asarray(det['position'], dtype=np.float64) for det in detections]
        det_boxes = [np.asarray(det['box'][:4], dtype=np.float64) for det in detections]

        if len(self._tracks) == 0:
            new_ids: List[int] = []
            for i, (det_pos, det_box) in enumerate(zip(det_positions, det_boxes)):
                track, _ = self._create_track(det_pos, det_box)
                if track is None:
                    break  # ID 池耗尽（不会发生在空 tracker 场景，但做防御）
                new_ids.append(track.track_id)
            if self.min_hits <= 1:
                return [TrackAssignment(track_id=tid, detection_index=i) for i, tid in enumerate(new_ids)], []
            return [], []

        track_ids = list(self._tracks.keys())
        cost_matrix = self._build_cost_matrix(track_ids, det_positions, det_boxes)

        # 匈牙利算法要求代价矩阵可行（每行/列至少有可用边）。
        # gating 过严时可能出现整行或整列都是 inf，这里先提取可行子矩阵。
        finite_mask = np.isfinite(cost_matrix)
        feasible_rows = np.where(np.any(finite_mask, axis=1))[0]
        feasible_cols = np.where(np.any(finite_mask, axis=0))[0]

        if feasible_rows.size == 0 or feasible_cols.size == 0:
            row_ind = np.array([], dtype=np.int64)
            col_ind = np.array([], dtype=np.int64)
        else:
            reduced_cost = cost_matrix[np.ix_(feasible_rows, feasible_cols)]
            try:
                row_sub, col_sub = linear_sum_assignment(reduced_cost)
                row_ind = feasible_rows[row_sub]
                col_ind = feasible_cols[col_sub]
            except ValueError:
                # 极端情况下（数值问题）降级为“本帧无匹配”，避免节点崩溃
                row_ind = np.array([], dtype=np.int64)
                col_ind = np.array([], dtype=np.int64)

        matched_track_ids = set()
        matched_det_ids = set()
        assignments: List[TrackAssignment] = []
        evicted_ids: List[int] = []

        for r, c in zip(row_ind, col_ind):
            cost = cost_matrix[r, c]
            if not np.isfinite(cost):
                continue

            track_id = track_ids[r]
            track = self._tracks[track_id]
            d2 = track.mahalanobis_squared(det_positions[c])
            if d2 > self.mahalanobis_thresh:
                continue

            track.update(det_positions[c], det_boxes[c])
            self._mark_mru(track_id)
            matched_track_ids.add(track_id)
            matched_det_ids.add(c)

            if track.hits >= self.min_hits:
                assignments.append(TrackAssignment(track_id=track_id, detection_index=c))

        unmatched_track_ids = [tid for tid in track_ids if tid not in matched_track_ids]
        unmatched_det_ids = [idx for idx in range(len(detections)) if idx not in matched_det_ids]

        for det_id in unmatched_det_ids:
            new_track, evicted_id = self._create_track(det_positions[det_id], det_boxes[det_id])
            if new_track is None:
                continue  # ID 池满且所有轨迹均活跃，丢弃该检测
            if evicted_id is not None:
                evicted_ids.append(evicted_id)
            if self.min_hits <= 1:
                assignments.append(TrackAssignment(track_id=new_track.track_id, detection_index=det_id))

        removed_ids = self._prune_specific_tracks(unmatched_track_ids)
        removed_ids.extend(evicted_ids)

        return assignments, removed_ids

    def _create_track(self, det_pos: np.ndarray, det_box: np.ndarray) -> Tuple[Optional[_KalmanTrack3D], Optional[int]]:
        """创建新轨迹，返回 (track, evicted_id)。

        若 ID 池耗尽，淘汰 LRU 轨迹并复用其 ID。
        若所有 16 个 ID 均在本帧被匹配，则拒绝创建（返回 None, None）。
        evicted_id 为被淘汰的 track_id，调用方应清理该 ID 的相关状态。
        """
        evicted_id: Optional[int] = None

        if self._available_ids:
            track_id = self._available_ids.pop(0)
        else:
            # LRU 淘汰：从队首开始找第一个本帧未匹配的轨迹
            evict_idx: Optional[int] = None
            for i, tid in enumerate(self._lru_order):
                if tid in self._tracks and self._tracks[tid].missed > 0:
                    evict_idx = i
                    break
            if evict_idx is None:
                # 所有 16 个 ID 都在活跃跟踪中，拒绝创建
                return None, None
            lru_id = self._lru_order.pop(evict_idx)
            if lru_id in self._tracks:
                del self._tracks[lru_id]
            evicted_id = lru_id
            track_id = lru_id

        track = _KalmanTrack3D(
            track_id=track_id,
            measurement_xyz=det_pos,
            bbox_xyxy=det_box,
            measurement_var=self.measurement_var,
        )
        self._tracks[track_id] = track
        self._mark_mru(track_id)
        return track, evicted_id

    def _mark_mru(self, track_id: int) -> None:
        """将 track_id 移到 LRU 队列队尾（标记为最近使用）。"""
        if track_id in self._lru_order:
            self._lru_order.remove(track_id)
        self._lru_order.append(track_id)

    def _prune_dead_tracks(self) -> List[int]:
        removed: List[int] = []
        for track_id in list(self._tracks.keys()):
            if self._tracks[track_id].missed > self.max_age:
                removed.append(track_id)
                self._release_track_id(track_id)
        return removed

    def _prune_specific_tracks(self, track_ids: Sequence[int]) -> List[int]:
        removed: List[int] = []
        for track_id in track_ids:
            if track_id not in self._tracks:
                continue
            if self._tracks[track_id].missed > self.max_age:
                removed.append(track_id)
                self._release_track_id(track_id)
        return removed

    def _release_track_id(self, track_id: int) -> None:
        """释放 track_id：归还到可用池并从 LRU 队列移除。"""
        if track_id in self._tracks:
            del self._tracks[track_id]
        if track_id in self._lru_order:
            self._lru_order.remove(track_id)
        if track_id not in self._available_ids:
            self._available_ids.append(track_id)

    def _build_cost_matrix(
        self,
        track_ids: Sequence[int],
        det_positions: Sequence[np.ndarray],
        det_boxes: Sequence[np.ndarray],
    ) -> np.ndarray:
        n_t = len(track_ids)
        n_d = len(det_positions)
        cost = np.full((n_t, n_d), np.inf, dtype=np.float64)

        pos_weight = max(0.0, min(1.0, 1.0 - self.iou_weight))
        iou_weight = max(0.0, min(1.0, self.iou_weight))

        for i, track_id in enumerate(track_ids):
            track = self._tracks[track_id]
            for j in range(n_d):
                d2 = track.mahalanobis_squared(det_positions[j])
                if d2 > self.mahalanobis_thresh:
                    continue

                pos_cost = d2 / self.mahalanobis_thresh
                iou = self._iou_xyxy(track.last_bbox, det_boxes[j])
                iou_cost = 1.0 - iou

                cost[i, j] = pos_weight * pos_cost + iou_weight * iou_cost

        return cost

    @staticmethod
    def _iou_xyxy(box_a: np.ndarray, box_b: np.ndarray) -> float:
        ax1, ay1, ax2, ay2 = box_a
        bx1, by1, bx2, by2 = box_b

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        area_a = max(0.0, (ax2 - ax1)) * max(0.0, (ay2 - ay1))
        area_b = max(0.0, (bx2 - bx1)) * max(0.0, (by2 - by1))
        union = area_a + area_b - inter_area

        if union <= 1e-9:
            return 0.0
        return float(inter_area / union)
