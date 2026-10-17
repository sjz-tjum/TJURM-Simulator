# core/bus/dataflow/prediction.py
"""预测消息：跟踪器给出的目标状态。

契约：
    - center / armors 表达系由 cfg.sim.frame 定（同 Observation）。
    - **armors[0] 是"未来时刻"的板位置**，不是当前板位置——tracker 已按
      total_delay 前推过。center 同理。
    - aim_rate = [yaw_rate, pitch_rate]（rad/s），是**枪口 → 瞄点视线**的角速度。
    - 一个 tracker 一帧可以发多条（按 target_type 分通道），第一条是"该打谁"。
    - 火控信息（fire / fire_mode）走 ShotSolution，不在本条里。
    - fps 不在本条里：它是 pipeline 层的度量，走 PIPELINE_STATS。
"""
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from core.entities.property.entity_types import TargetType


@dataclass
class GimbalTarget:
    yaw: float
    yaw_rate: float
    yaw_acc: float
    pitch: float
    pitch_rate: float
    pitch_acc: float


@dataclass
class Prediction:
    target_type: TargetType
    center: np.ndarray
    armors: List[np.ndarray]
    timestamp: float
    is_tracking: bool
    aim_rate: Optional[np.ndarray] = None
    gimbal_target: Optional[GimbalTarget] = None