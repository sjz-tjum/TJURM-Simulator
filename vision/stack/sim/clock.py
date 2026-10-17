# sim/clock.py
"""仿真时钟：唯一时基，单位永远是秒。"""
from __future__ import annotations


class SimClock:
    def __init__(self):
        self.sim_time = 0.0
        self.dt = 0.0

    def update_from_ms(self, elapsed_ms: float) -> float:
        """实时模式：pgcompat.Clock.tick() 返回毫秒，这里转成秒。"""
        self.dt = float(elapsed_ms) / 1000.0
        self.sim_time += self.dt
        return self.dt

    def step(self, dt: float) -> float:
        """固定步长：不读墙钟，只加 dt。"""
        self.dt = float(dt)
        self.sim_time += self.dt
        return self.dt

    def reset(self) -> None:
        self.sim_time = 0.0
        self.dt = 0.0