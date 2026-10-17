# core/bus/dataflow/control.py
"""自瞄发出的火控指令：瞄准角度 + 前馈 + 是否开火 + 开火模式 + 目标身份 + 冷却建议。

契约（流水发弹模型）：
    - yaw / pitch 是**绝对系**下的云台指向；pitch **抬头为正**。
    - yaw_rate / pitch_rate 是瞄点视线的角速度（rad/s），给电控做前馈。
    - fire 是自瞄的推荐："这一帧该不该扣扳机"。
    - fire_mode: "auto"（连发）/ "single"（单发）。
    - fire_key: 自瞄估计的"当前瞄的板/灯臂编号"；虚拟点时为 None。
    - cooldown: 自瞄建议的最小两发间隔（秒）。0 = 用电控的默认。
      符是 0.8s（灯臂亮 2.5s 窗口内打 3 发）；车 / 前哨留 0。
    - timestamp 必须与产生它的那条 Prediction 一致。
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class ShotSolution:
    yaw: float
    pitch: float
    yaw_rate: float = 0.0
    pitch_rate: float = 0.0
    fire: bool = False
    fire_mode: str = "auto"
    fire_key: Optional[int] = None
    cooldown: float = 0.0
    timestamp: float = 0.0