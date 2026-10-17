# core/bus/dataflow/detection.py
"""检测消息：传感器一帧内看到的全部目标，**只有 2D 图像点**。

契约：
    - Detection.image_points_2d 恒为 (4,2)，顺序 LT / RT / RB / LB。
    - Detection.center 仅圆靶有值（非圆靶为 None）。
    - 不含目标身份：谁看到哪块板由下游自己维护。
    - DetectionFrame 空帧也发（detections=[]），告诉下游"这一拍什么都没看到"。
"""
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from core.entities.property.entity_types import EntityType


@dataclass
class Detection:
    entity_type: EntityType
    image_points_2d: np.ndarray
    armor_size: str = ""
    lit: bool = False
    center: Optional[np.ndarray] = None


@dataclass
class DetectionFrame:
    detections: List[Detection] = field(default_factory=list)
    timestamp: float = 0.0