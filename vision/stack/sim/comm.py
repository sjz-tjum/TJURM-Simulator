# sim/comm.py
"""sim 侧通信模块：simulator 每帧调它。

发：检测 + 自车状态（只在传感器出帧时发，两者同刻配对）
收：控制指令（缓存最新一份，电控每帧取）
"""
from __future__ import annotations

from core.bus import public_bus
from core.bus.messages import PublicMessages


class SimComm:
    def __init__(self):
        self._latest_control = None
        public_bus().subscribe(PublicMessages.CONTROL, self._on_control)

    def _on_control(self, solution):
        self._latest_control = solution

    # ------------------------------------------------------------ 发
    def publish_frame(self, frame, self_state):
        """sim 侧：发一帧检测 + 同一刻的自车状态。"""
        public_bus().publish(PublicMessages.DETECTION, frame)
        public_bus().publish(PublicMessages.SELF_STATE, self_state)

    # ------------------------------------------------------------ 收
    def latest_control(self):
        """sim 侧：取 custom 发来的最新控制指令（可能为 None）。"""
        return self._latest_control

    def reset(self):
        self._latest_control = None