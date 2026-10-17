# sim/render/viewport.py
"""Viewport：屏幕、面板矩形、绘制原语。

管理 pygame 屏幕和四块面板，提供屏幕级绘制原语。
不知道仿真实体、不知道总线数据、不知道业务语义。
"""
from __future__ import annotations

import math

import numpy as np
import pygame as pg


class Viewport:
    def __init__(self, ui):
        self.screen_width = int(ui.screen_width)
        self.screen_height = int(ui.screen_height)
        self.screen = pg.display.set_mode((self.screen_width, self.screen_height))

        self.main_rect = pg.Rect(*ui.panels.main.rect)
        self.camera_rect = pg.Rect(*ui.panels.camera.rect)
        self.info_rect = pg.Rect(*ui.panels.info.rect)
        self.curve_rect = pg.Rect(*ui.panels.curve.rect)

        self.main_center = np.array(self.main_rect.center, float)
        self.camera_center = np.array(self.camera_rect.center, float)

        # 复用的半透明面（主视图 / 相机视图用）
        self.main_alpha = pg.Surface(
            (self.main_rect.width, self.main_rect.height), pg.SRCALPHA)
        self.camera_alpha = pg.Surface(
            (self.camera_rect.width, self.camera_rect.height), pg.SRCALPHA)

    # ---------------------------------------------------------------- 裁剪
    def set_clip(self, rect):
        self.screen.set_clip(rect)

    def clear_clip(self):
        self.screen.set_clip(None)

    # ---------------------------------------------------------------- 原语
    def draw_point(self, screen_pos, color, radius):
        pg.draw.circle(self.screen, color, screen_pos, radius)

    def draw_hit_cross(self, center, color):
        cx, cy = int(center[0]), int(center[1])
        pg.draw.line(self.screen, color, (cx - 8, cy - 8), (cx + 8, cy + 8), 3)
        pg.draw.line(self.screen, color, (cx + 8, cy - 8), (cx - 8, cy + 8), 3)

    def draw_arrow_head(self, start, end, color, width):
        start = (int(start[0]), int(start[1]))
        end = (int(end[0]), int(end[1]))
        if start == end:
            return
        pg.draw.line(self.screen, color, start, end, width)
        angle = math.atan2(end[1] - start[1], end[0] - start[0])
        for offset in (math.radians(150.0), math.radians(-150.0)):
            head = (int(end[0] + 8 * math.cos(angle + offset)),
                    int(end[1] + 8 * math.sin(angle + offset)))
            pg.draw.line(self.screen, color, end, head, width)

    def fill_polygon_alpha(self, surf, rect, rel_points, rgba):
        """在复用的透明面上填多边形，只 blit 包围盒。"""
        points = [(int(p[0]), int(p[1])) for p in rel_points]
        if not points:
            return
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        x0 = max(min(xs) - 2, 0)
        y0 = max(min(ys) - 2, 0)
        x1 = min(max(xs) + 3, surf.get_width())
        y1 = min(max(ys) + 3, surf.get_height())
        if x1 <= x0 or y1 <= y0:
            return
        area = pg.Rect(x0, y0, x1 - x0, y1 - y0)
        surf.fill((0, 0, 0, 0), area)
        pg.draw.polygon(surf, rgba, points)
        self.screen.blit(surf, (rect.left + x0, rect.top + y0), area)