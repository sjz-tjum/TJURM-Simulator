# core/bus/dataflow/observation.py
"""解算后的观测消息：一帧内全部目标的位姿。

契约：
    - rel_pos 表达系由 cfg.sim.frame 定：G = 相对**云台中心**，W = 绝对世界。
    - rel_rpy[2] 是板在**绝对系**下的偏航 psi（不是相对云台的板法线角）。
    - rel_rpy[1] = 板在绝对系下的俯仰 = 安装俯仰（平地假设）。
    - 空帧也发（obs_armors=[]）。
    - 消息里没有目标身份：谁看到哪块板由 tracker 的观测匹配自己维护。
"""
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from core.entities.property.entity_types import EntityType


@dataclass
class ArmorObservation:
    entity_type: EntityType
    rel_pos: np.ndarray
    rel_rpy: np.ndarray
    armor_size: str = ""
    image_points_2d: Optional[np.ndarray] = None
    lit: bool = False


@dataclass
class Observation:
    obs_armors: List[ArmorObservation] = field(default_factory=list)
    timestamp: float = 0.0