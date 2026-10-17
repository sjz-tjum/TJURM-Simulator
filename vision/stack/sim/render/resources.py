# sim/render/resources.py
"""Resources：调色板 + 字体缓存。"""
from __future__ import annotations

import pygame as pg

_CJK_FONT_CANDIDATES = ("microsoftyahei", "simhei", "simsun", "dengxian")

#: ui.yaml 里没有对应键的两项
_EXTRA_CAMERA_VIOLET = (100, 100, 255)
_PRED_CENTER = (255, 0, 0)


class Palette:
    """配色：能进 ui.yaml 的读 ui.yaml，进不去的硬编码。"""

    def __init__(self, ui_colors):
        rgb = lambda values: tuple(int(v) for v in values)
        self.white = (255, 255, 255)
        self.robot_alive = rgb(ui_colors.robot_alive)
        self.robot_dead = rgb(ui_colors.robot_dead)
        self.armor = rgb(ui_colors.armor)
        self.observation = rgb(ui_colors.observation)
        self.prediction = rgb(ui_colors.prediction)
        self.bullet = rgb(ui_colors.bullet)
        self.hit = rgb(ui_colors.hit)
        self.camera = rgb(ui_colors.camera)
        self.rune_activatable = rgb(ui_colors.rune_activatable)
        self.rune_activated = rgb(ui_colors.rune_activated)
        self.camera_other = _EXTRA_CAMERA_VIOLET
        self.pred_center = _PRED_CENTER


class Resources:
    def __init__(self, ui_colors):
        self.c = Palette(ui_colors)
        self._fonts = {}

    def info_font(self, size):
        font = self._fonts.get(size)
        if font is None:
            font = _load_cjk_font(size)
            self._fonts[size] = font
        return font


def _load_cjk_font(size):
    """挑一个能画出中文的字体；宽度明显小于字号说明只是缺字形方块。"""
    for name in _CJK_FONT_CANDIDATES:
        path = pg.font.match_font(name)
        if not path:
            continue
        try:
            font = pg.font.Font(path, size)
        except Exception:
            continue
        if font.size("中")[0] >= size * 0.6:
            return font
    return pg.font.Font(None, size)