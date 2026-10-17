# core/bus/dataflow/self_state.py
"""自车状态：一帧内受控机器人的全部关键位姿。

契约：
    - 所有 pos 都是**世界系绝对坐标**（不是 G 系相对量）。
    - 消费者要 G 系相对量，自己减 gimbal.pos。
    - K 是相机内参矩阵（3x3）。
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class RigidState:
    pos: np.ndarray       # (3,)
    rpy: np.ndarray       # (3,)
    vel: np.ndarray       # (3,)
    omg: np.ndarray       # (3,)


@dataclass
class CameraState:
    pos: np.ndarray       # (3,)
    rpy: np.ndarray       # (3,)
    K: np.ndarray         # (3,3)


@dataclass
class SelfState:
    chassis: RigidState
    gimbal: RigidState
    muzzle: RigidState
    camera: CameraState