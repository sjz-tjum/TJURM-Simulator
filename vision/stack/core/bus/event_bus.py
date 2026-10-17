# core/bus/event_bus.py
"""事件总线：线程安全的发布订阅类，不含全局实例。

topic 参数接受两种形式：
    Message  —— 标准消息（core 定义）
    str      —— 自定义 topic（custom 自由使用）
两者对 EventBus 是同一种东西，都是字符串键。
"""
from __future__ import annotations

import sys
import threading
import traceback
from collections import defaultdict
from typing import Any, Callable, Dict, List


def _topic_of(topic) -> str:
    """Message -> topic 名；裸字符串原样返回。"""
    return getattr(topic, "topic", topic)


class EventBus:
    def __init__(self) -> None:
        self._subscribers: Dict[str, List[Callable[[Any], None]]] = defaultdict(list)
        self._lock = threading.Lock()

    def subscribe(self, topic, callback: Callable[[Any], None]) -> None:
        with self._lock:
            self._subscribers[_topic_of(topic)].append(callback)

    def unsubscribe(self, topic, callback: Callable[[Any], None]) -> None:
        with self._lock:
            key = _topic_of(topic)
            if key in self._subscribers:
                try:
                    self._subscribers[key].remove(callback)
                except ValueError:
                    pass

    def publish(self, topic, data: Any) -> None:
        key = _topic_of(topic)
        with self._lock:
            callbacks = list(self._subscribers.get(key, ()))
        for cb in callbacks:
            try:
                cb(data)
            except Exception:
                print(f"[EventBus] {key!r} 回调异常：", file=sys.stderr)
                traceback.print_exc()

    def clear(self) -> None:
        with self._lock:
            self._subscribers.clear()

    def __repr__(self) -> str:
        return f"EventBus(topics={list(self._subscribers)})"