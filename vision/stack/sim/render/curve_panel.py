# sim/render/curve_panel.py
"""CurvePanel：上偏航、下俯仰的轻量折线。

数据源契约：

    cmd_yaw  / cmd_pitch   <- controlled.gimbal.target_theta / target_pitch（渲染器直读实体）
    act_yaw  / act_pitch   <- controlled.gimbal.world_rpy[2] / world_rpy[1]
    obs_yaw  / obs_pitch   <- DEBUG_PLOT 的 'obs_yaw' / 'obs_pitch'
    pred_yaw / pred_pitch  <- DEBUG_PLOT 的 'pred_yaw' / 'pred_pitch'

后四项由 tracker 通过 debug_plot(**values) 发；缺失填 nan，非有限值沿用上一帧
（零阶保持），否则 tracker 没数据时曲线会断崖。
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np
import pygame as pg

_BORDER = (100, 106, 114)
_BG = (14, 16, 20, 210)
_AXIS_BG = (32, 36, 42)
_AXIS_BORDER = (70, 76, 84)
_ZERO = (58, 62, 70)
_TEXT = (150, 158, 168)
_TITLE_FG = (230, 236, 244)

_PRESETS = (
    ("cmd vs act", ("cmd", "act")),
    ("cmd/act + obs", ("cmd", "act", "obs")),
    ("cmd/act + pred", ("cmd", "act", "pred")),
    ("all four", ("cmd", "act", "obs", "pred")),
)
_COLORS = {"cmd": (255, 165, 0), "act": (0, 255, 0),
           "obs": (0, 255, 255), "pred": (255, 0, 255)}

_CURVE_LEN = 300
_NAMES = ("cmd", "act", "obs", "pred")


class CurvePanel:
    def __init__(self, viewport, resources):
        self.viewport = viewport
        self.resources = resources
        self.visible = False
        self.preset = 0
        self.curves = {f"{n}_{axis}": deque(maxlen=_CURVE_LEN)
                       for n in _NAMES for axis in ("yaw", "pitch")}
        self._plot_latest = {}
        self._alpha_surface = None

    # ---------------------------------------------------------------- 状态
    def switch_visible(self):
        self.visible = not self.visible
        return self.visible

    def cycle_preset(self):
        self.preset = (self.preset + 1) % len(_PRESETS)
        return self.preset

    def reset(self):
        for series in self.curves.values():
            series.clear()
        self.visible = False
        self.preset = 0
        self._plot_latest = {}

    # ---------------------------------------------------------------- 采样
    def sample(self, plot, controlled):
        """每帧采一次（不管可见与否，保证曲线不断崖）。"""
        gimbal = controlled.gimbal if controlled is not None else None
        self._plot_latest = plot or {}

        def deg(v):
            try:
                v = float(v)
            except (TypeError, ValueError):
                return float("nan")
            return math.degrees(v) if math.isfinite(v) else float("nan")

        values = {
            "cmd_yaw": deg(gimbal.target_theta) if gimbal is not None else np.nan,
            "act_yaw": deg(gimbal.world_rpy[2]) if gimbal is not None else np.nan,
            "cmd_pitch": deg(gimbal.target_pitch) if gimbal is not None else np.nan,
            "act_pitch": deg(gimbal.world_rpy[1]) if gimbal is not None else np.nan,
            "obs_yaw": deg(self._plot_latest.get("obs_yaw", np.nan)),
            "obs_pitch": deg(self._plot_latest.get("obs_pitch", np.nan)),
            "pred_yaw": deg(self._plot_latest.get("pred_yaw", np.nan)),
            "pred_pitch": deg(self._plot_latest.get("pred_pitch", np.nan)),
        }
        for key, series in self.curves.items():
            value = values.get(key, np.nan)
            if series and not np.isfinite(value):
                value = series[-1]
            series.append(value)

    # ---------------------------------------------------------------- 渲染
    def draw(self):
        vp = self.viewport
        panel = vp.curve_rect
        title, names = _PRESETS[self.preset]

        if (self._alpha_surface is None
                or self._alpha_surface.get_size() != (panel.width, panel.height)):
            self._alpha_surface = pg.Surface((panel.width, panel.height), pg.SRCALPHA)
        self._alpha_surface.fill(_BG)
        vp.screen.blit(self._alpha_surface, panel.topleft)
        pg.draw.rect(vp.screen, _BORDER, panel, 2)

        font = self.resources.info_font(15)
        head = f"CURVES  [M hide | KP4 preset: {title}]"
        vp.screen.blit(font.render(head, True, _TITLE_FG),
                       (panel.left + 8, panel.top + 2))

        sub_height = (panel.height - 26) // 2
        for k, axis in enumerate(("yaw", "pitch")):
            sub = pg.Rect(panel.left + 8, panel.top + 22 + k * sub_height,
                          panel.width - 16, sub_height - 6)
            self._draw_axis(sub, font, axis, names)

    def _draw_axis(self, rect, font, axis, names):
        vp = self.viewport
        pg.draw.rect(vp.screen, _AXIS_BG, rect)
        pg.draw.rect(vp.screen, _AXIS_BORDER, rect, 1)

        series = [(name, list(self.curves[f"{name}_{axis}"])) for name in names]
        finite = [v for _n, values in series for v in values if np.isfinite(v)]
        if not finite:
            vp.screen.blit(font.render(f"{axis}: no data", True, _TEXT),
                           (rect.left + 6, rect.top + 4))
            return
        lo, hi = min(finite), max(finite)
        pad = max((hi - lo) * 0.15, 0.5)
        lo, hi = lo - pad, hi + pad

        plot_w = rect.width - 56
        count = max(len(values) for _n, values in series)
        if lo < 0.0 < hi:
            zero_y = rect.bottom - 4 - (rect.height - 10) * (0.0 - lo) / (hi - lo)
            pg.draw.line(vp.screen, _ZERO, (rect.left + 2, zero_y),
                         (rect.left + plot_w, zero_y), 1)

        for name, values in series:
            points = []
            for i, v in enumerate(values):
                if not np.isfinite(v):
                    continue
                x = rect.left + 2 + plot_w * i / max(count - 1, 1)
                y = rect.bottom - 4 - (rect.height - 10) * (v - lo) / (hi - lo)
                points.append((int(x), int(y)))
            if len(points) >= 2:
                pg.draw.lines(vp.screen, _COLORS[name], False, points, 2)

        label_x = rect.left + plot_w + 4
        vp.screen.blit(font.render(f"{hi:5.1f}", True, _TEXT), (label_x, rect.top + 1))
        vp.screen.blit(font.render(f"{lo:5.1f}", True, _TEXT),
                       (label_x, rect.bottom - 15))
        for name, values in series:
            last = next((v for v in reversed(values) if np.isfinite(v)), np.nan)
            text = (f"{axis} {name} {last:6.1f}" if np.isfinite(last)
                    else f"{axis} {name}   --")
            vp.screen.blit(font.render(text, True, _COLORS[name]),
                           (rect.left + 6 + (0 if name in ("cmd", "obs") else 150),
                            rect.top + 2))