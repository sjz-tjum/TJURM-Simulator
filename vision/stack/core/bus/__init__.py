# core/bus/__init__.py
"""总线层。

两条总线，按"谁在系统侧"划：

    sys_bus()     系统侧（sim + main/entrance）内部的总线，custom 不碰。
    public_bus()  系统侧（sim/main）与 custom 之间的**唯一**通道。

    EventBus          —— 通用发布订阅类。
    Message           —— 一个 topic 绑定一个 payload 类型。
    SysMessages       —— sys_bus 的标准消息。
    PublicMessages    —— public_bus 的标准消息。

便捷函数（都走 public_bus）：

    publish(topic, data)
    subscribe(topic, callback)
    unsubscribe(topic, callback)
    debug_text(text, color, view)
    debug_plot(**values)

测试用替换：

    install_sys_bus(bus)     / reset_sys_bus()
    install_public_bus(bus)  / reset_public_bus()
"""
from core.bus.event_bus import EventBus
from core.bus.sys_bus import (
    sys_bus, install_sys_bus, reset_sys_bus,
)
from core.bus.public_bus import (
    public_bus, install_public_bus, reset_public_bus,
    publish, subscribe, unsubscribe,
    debug_text, debug_plot,
)
from core.bus.messages import Message, SysMessages, PublicMessages

__all__ = [
    "EventBus",
    "sys_bus", "install_sys_bus", "reset_sys_bus",
    "public_bus", "install_public_bus", "reset_public_bus",
    "publish", "subscribe", "unsubscribe",
    "debug_text", "debug_plot",
    "Message", "SysMessages", "PublicMessages",
]