# sim/render/bus_cache.py
"""BusCache：订阅 public_bus，缓存最新消息，提供一致快照。"""
from __future__ import annotations

import threading
from collections import deque

from core.bus import public_bus
from core.bus.dataflow import DrawText, Observation, PipelineStats, Prediction
from core.bus.messages import PublicMessages
from core.entities.property.entity_types import TargetType


class BusCache:
    def __init__(self):
        self._latest_obs = None
        #: 三条 pred 主题各一槽：挤进同一槽会互相覆盖
        self._latest_preds = {t: None for t in TargetType}
        self._last_pred_type = None
        self._latest_stats = PipelineStats()
        self._draw_texts = deque(maxlen=10)
        self._plot_latest = {}
        self._lock = threading.Lock()

        bus = public_bus()
        bus.subscribe(PublicMessages.OBS, self._on_obs)
        for t in TargetType:
            bus.subscribe(PublicMessages.pred(t), self._make_pred_handler(t))
        bus.subscribe(PublicMessages.PIPELINE_STATS, self._on_stats)
        bus.subscribe(PublicMessages.DEBUG_TEXT, self._on_draw)
        bus.subscribe(PublicMessages.DEBUG_PLOT, self._on_plot)

    # ---------------------------------------------------------------- 回调
    def _make_pred_handler(self, target_type):
        """三条 pred 主题各绑一个闭包：只有闭包知道回调来自哪条通道。"""
        def handler(data):
            self._on_pred(target_type, data)
        return handler

    def _on_obs(self, data: Observation) -> None:
        with self._lock:
            self._latest_obs = data

    def _on_pred(self, target_type, data: Prediction) -> None:
        with self._lock:
            self._latest_preds[target_type] = data
            self._last_pred_type = target_type

    def _on_stats(self, data: PipelineStats) -> None:
        with self._lock:
            self._latest_stats = data

    def _on_draw(self, data: DrawText) -> None:
        with self._lock:
            self._draw_texts.append(data)

    def _on_plot(self, data) -> None:
        with self._lock:
            self._plot_latest = dict(data) if isinstance(data, dict) else {}

    # ---------------------------------------------------------------- 快照
    def snapshot(self):
        """一帧只取一次，别画到一半被跟踪线程换掉数据。"""
        with self._lock:
            obs = self._latest_obs
            pred = self._latest_preds.get(self._last_pred_type)
            stats = self._latest_stats
            plot = dict(self._plot_latest)
            texts = list(self._draw_texts)
        return obs, pred, stats, plot, texts

    def reset(self):
        with self._lock:
            self._latest_obs = None
            for t in self._latest_preds:
                self._latest_preds[t] = None
            self._last_pred_type = None
            self._latest_stats = PipelineStats()
            self._draw_texts.clear()
            self._plot_latest = {}