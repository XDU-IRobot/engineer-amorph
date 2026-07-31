"""
位姿平滑器
使用指数移动平均(EMA)对位置进行滤波，使用球面线性插值(SLERP)对姿态进行平滑
"""

from typing import Tuple, List, Optional, Dict
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class PoseSmoother:
    """
    位姿平滑器，对位置和姿态进行平滑滤波
    """
    
    def __init__(self, alpha: float = 0.3):
        """
        初始化位姿平滑器
        
        参数:
            alpha: 平滑因子 [0, 1]，越小越平滑，越大响应越快
                  alpha=1 时完全不平滑（直接使用当前值）
                  alpha=0 时完全不更新（始终保持初始值）
        """
        self.alpha = alpha
        self.states: Dict[int, Dict[str, np.ndarray]] = {}
    
    def update(self, 
               unit_id: int, 
               position: List[float], 
               orientation: List[float]) -> Tuple[List[float], List[float]]:
        """
        更新并返回平滑后的位姿
        
        参数:
            unit_id: 目标对象ID（用于跟踪不同对象的状态）
            position: 当前位置 [x, y, z]
            orientation: 当前姿态四元数 [x, y, z, w]
            
        返回:
            (smoothed_position, smoothed_orientation)
            smoothed_position: 平滑后的位置 [x, y, z]
            smoothed_orientation: 平滑后的姿态四元数 [x, y, z, w]
        """
        if unit_id not in self.states:
            # 首次检测到该对象，直接使用当前值初始化
            self.states[unit_id] = {
                'position': np.array(position, dtype=np.float64),
                'orientation': np.array(orientation, dtype=np.float64)
            }
            return position, orientation
        
        state = self.states[unit_id]
        
        # 位置平滑：EMA滤波
        smoothed_pos = self._smooth_position(
            state['position'], 
            np.array(position, dtype=np.float64)
        )
        
        # 姿态平滑：SLERP插值
        smoothed_ori = self._smooth_orientation(
            state['orientation'], 
            np.array(orientation, dtype=np.float64)
        )
        
        # 更新状态
        state['position'] = smoothed_pos
        state['orientation'] = smoothed_ori
        
        return smoothed_pos.tolist(), smoothed_ori.tolist()
    
    def _smooth_position(self, 
                        prev_position: np.ndarray, 
                        curr_position: np.ndarray) -> np.ndarray:
        """
        使用指数移动平均(EMA)平滑位置
        
        公式: smoothed = (1 - alpha) * prev + alpha * current
        """
        return (1 - self.alpha) * prev_position + self.alpha * curr_position
    
    def _smooth_orientation(self, 
                           prev_quat: np.ndarray, 
                           curr_quat: np.ndarray) -> np.ndarray:
        """
        使用球面线性插值(SLERP)平滑姿态
        
        SLERP保证了四元数插值的最短路径和恒定角速度
        """
        r_prev = Rotation.from_quat(prev_quat)
        r_curr = Rotation.from_quat(curr_quat)
        
        # 创建SLERP插值器，在0和1之间插值
        slerp = Slerp([0, 1], Rotation.concatenate([r_prev, r_curr]))
        
        # 使用alpha作为插值因子
        smoothed_rotation = slerp(self.alpha)
        
        return smoothed_rotation.as_quat()
    
    def reset(self, unit_id: Optional[int] = None):
        """
        重置平滑器状态
        
        参数:
            unit_id: 要重置的对象ID。如果为None，则重置所有对象
        """
        if unit_id is None:
            self.states.clear()
        elif unit_id in self.states:
            del self.states[unit_id]
    
    def set_alpha(self, alpha: float):
        """
        动态调整平滑因子
        
        参数:
            alpha: 新的平滑因子 [0, 1]
        """
        if not 0 <= alpha <= 1:
            raise ValueError(f"Alpha must be in [0, 1], got {alpha}")
        self.alpha = alpha
    
    def get_state(self, unit_id: int) -> Optional[Dict[str, np.ndarray]]:
        """
        获取指定对象的当前平滑状态
        
        参数:
            unit_id: 对象ID
            
        返回:
            包含 'position' 和 'orientation' 的字典，如果对象不存在则返回None
        """
        return self.states.get(unit_id)
    
    def has_state(self, unit_id: int) -> bool:
        """
        检查是否存在指定对象的状态
        
        参数:
            unit_id: 对象ID
            
        返回:
            如果存在该对象的状态则返回True
        """
        return unit_id in self.states
