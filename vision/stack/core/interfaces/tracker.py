# core/interfaces/tracker.py
"""Tracker 接口：MOT 方案。消费 Observation，产出 Prediction 列表。

实现在 custom/schemes 下；sim/pipeline 只通过这个接口调用它。
"""
from __future__ import annotations

from typing import List

from core.bus.dataflow import Prediction
from core.interfaces.registry import Registry, register

__all__ = ["Tracker", "register_tracker"]


class Tracker(Registry):
    kind = "跟踪器"

    def step(self, obs, self_state) -> List[Prediction]:
        raise NotImplementedError

    def reset(self) -> None:
        pass


def register_tracker(name, aliases=None):
    return register(Tracker, name, aliases)