# core/bus/messages.py
"""消息契约：一个 topic 绑定一个 dataflow 类型。

core 定义标准 message；custom 可以自由地用任意字符串作 topic 传自定义数据。
EventBus 不区分"标准 message"和"自定义字符串"——它的参数接受两者。

同名 topic 可能出现在两条总线上（如 detection 既是 sys 也是 public），
它们是两条独立通道上的两个 message，语义相同、通道不同。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Type

from core.bus.dataflow import (DetectionFrame, DrawText, Observation,
                               PipelineStats, Prediction, SelfState,
                               ShotSolution)


@dataclass(frozen=True)
class Message:
    """一个总线消息：topic 名 + 对应的 dataflow 类型。

    dataflow=None 表示无固定类型；dataflow=dict 表示自由字段。
    """
    topic: str
    dataflow: Optional[Type] = None


class SysMessages:
    """sim 内部总线（sys_bus）的标准消息。custom 不碰这条总线。"""

    DETECTION = Message("detection", DetectionFrame)
    SELF_STATE = Message("self_state", SelfState)
    RESET = Message("reset", None)      # dataflow 忽略
    FIRE = Message("fire", dict)        # 自由字段


class PublicMessages:
    """公开总线（public_bus）的标准消息。sim 与 custom 都通过它通信。"""

    DETECTION = Message("detection", DetectionFrame)
    SELF_STATE = Message("self_state", SelfState)

    OBS = Message("obs", Observation)
    CONTROL = Message("control", ShotSolution)

    DEBUG_TEXT = Message("debug/text", DrawText)
    DEBUG_PLOT = Message("debug/plot", dict)
    PIPELINE_STATS = Message("pipeline/stats", PipelineStats)

    @staticmethod
    def pred(target_type) -> Message:
        """pred/<robot|outpost|rune>，dataflow 为 Prediction。"""
        return Message(f"pred/{target_type.value}", Prediction)