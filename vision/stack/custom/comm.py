# custom/comm.py
"""custom 侧通信模块：pipeline.tick 调它。

收：检测 + 自车状态（缓存最新一份）
发：控制指令
"""
from __future__ import annotations

from core.bus import public_bus
from core.bus.messages import PublicMessages


class CustomComm:
    def __init__(self):
        self._latest_frame = None
        self._latest_state = None
        public_bus().subscribe(PublicMessages.DETECTION, self._on_frame)
        public_bus().subscribe(PublicMessages.SELF_STATE, self._on_state)

    def _on_frame(self, frame):
        self._latest_frame = frame

    def _on_state(self, state):
        self._latest_state = state

    # ------------------------------------------------------------ 收
    def latest_inputs(self):
        """custom 侧：取最新一帧的 (检测, 自车状态)。"""
        return self._latest_frame, self._latest_state

    # ------------------------------------------------------------ 发
    def publish_control(self, solution):
        """custom 侧：发控制指令。"""
        public_bus().publish(PublicMessages.CONTROL, solution)

    def reset(self):
        self._latest_frame = None
        self._latest_state = None