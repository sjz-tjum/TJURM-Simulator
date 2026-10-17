# core/bus/public_bus.py
"""公开总线：sim 和 custom 共享。调试、绘图、日志、遥测。

任何地方都能发/收。控制信号不走这里。
"""
from __future__ import annotations

from typing import Any, Callable

from core.bus.event_bus import EventBus
from core.bus.messages import PublicMessages

_public_bus: EventBus | None = None


# ---------------------------------------------------------------- 访问点
def public_bus() -> EventBus:
    global _public_bus
    if _public_bus is None:
        _public_bus = EventBus()
    return _public_bus


def install_public_bus(bus: EventBus) -> None:
    """替换公开总线实例（测试、多实例）。"""
    global _public_bus
    _public_bus = bus


def reset_public_bus() -> None:
    global _public_bus
    if _public_bus is not None:
        _public_bus.clear()
    _public_bus = None


# ------------------------------------------------------------ 便捷函数
def publish(topic, data: Any) -> None:
    public_bus().publish(topic, data)


def subscribe(topic, callback: Callable[[Any], None]) -> None:
    public_bus().subscribe(topic, callback)


def unsubscribe(topic, callback: Callable[[Any], None]) -> None:
    public_bus().unsubscribe(topic, callback)


# ------------------------------------------------------------ 调试便捷函数
def debug_text(text: str,
               color: tuple[int, int, int] = (255, 255, 255),
               view: str = "info") -> None:
    """发一条文本到调试面板。"""
    from core.bus.dataflow.draw import DrawText
    publish(PublicMessages.DEBUG_TEXT,
            DrawText(text=text, color=color, view=view))


def debug_plot(**values: float) -> None:
    """发一组数值到曲线面板。"""
    publish(PublicMessages.DEBUG_PLOT, dict(values))