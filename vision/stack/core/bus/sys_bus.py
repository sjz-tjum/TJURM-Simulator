# core/bus/sys_bus.py
"""系统内部总线：只有 sim 内部的 Manager 使用。

约定：custom 不发布、不订阅此总线。
如果以后想让 sim 拥有私有总线（避免 custom 误用），
可以不走这里的全局访问点，改成 Simulator 自己持有 EventBus 并注入。
"""
from __future__ import annotations

from core.bus.event_bus import EventBus

_sys_bus: EventBus | None = None


def sys_bus() -> EventBus:
    global _sys_bus
    if _sys_bus is None:
        _sys_bus = EventBus()
    return _sys_bus


def install_sys_bus(bus: EventBus) -> None:
    """替换系统总线实例（测试、多实例）。"""
    global _sys_bus
    _sys_bus = bus


def reset_sys_bus() -> None:
    global _sys_bus
    if _sys_bus is not None:
        _sys_bus.clear()
    _sys_bus = None