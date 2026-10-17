# sim/render/info_panel.py
"""InfoPanel：观测 / 跟踪 / 血量 / 能量机关 / 调试文本。

字号与列数自适应：先按展开后的实际行数排版，行数超出就缩小字号。
"""
from __future__ import annotations

import numpy as np
import pygame as pg

_BG = (26, 28, 32)
_BORDER = (96, 102, 112)
_TITLE_BG = (36, 40, 46)
_TITLE_FG = (230, 236, 244)
_LABEL_FG = (150, 158, 168)
_HEADER_FG = (120, 200, 220)
_RULE = (70, 76, 84)
_EVENT_FG = (200, 200, 200)


class InfoPanel:
    def __init__(self, viewport, resources):
        self.viewport = viewport
        self.resources = resources

    def reset(self):
        pass

    # ---------------------------------------------------------------- 主入口
    def draw(self, robots, outpost, rune, obs, pred, stats, draw_texts):
        vp = self.viewport
        rect = vp.info_rect
        c = self.resources.c

        pg.draw.rect(vp.screen, _BG, rect)
        pg.draw.rect(vp.screen, _BORDER, rect, 2)

        rows = self._build_rows(robots, outpost, rune, obs, pred, stats, draw_texts)
        font, lines_per_row, col_count, title_height, label_width, col_width, line_height = \
            self._layout(rows, rect)

        self._draw_title_bar(rect, font, title_height, stats)
        self._draw_rows(rect, font, rows, lines_per_row, col_count,
                        title_height, label_width, col_width, line_height)

    # ---------------------------------------------------------------- 行构建
    def _build_rows(self, robots, outpost, rune, obs, pred, stats, draw_texts):
        c = self.resources.c
        rows = []

        def header(title):
            rows.append(("header", title, "", _HEADER_FG))

        def kv(label, value, color=None):
            rows.append(("kv", label, str(value),
                         c.white if color is None else color))

        def text(s, color=None):
            rows.append(("text", "", str(s),
                         c.white if color is None else color))

        header("VISION")
        kv("obs armors", len(obs.obs_armors) if obs is not None else 0, c.armor)

        header("TRACK")
        if pred is not None:
            kv("target", pred.target_type.value, c.prediction)
            kv("tracking", "yes" if pred.is_tracking else "no",
               c.bullet if pred.is_tracking else c.hit)
            if pred.center is not None and len(pred.center) >= 3:
                kv("center", f"({pred.center[0]:.3f}, {pred.center[1]:.3f}, "
                             f"{pred.center[2]:.3f})")
            else:
                kv("center", "none", c.robot_dead)
            kv("armors", len(pred.armors) if pred.armors else 0, c.bullet)
        kv("pipeline fps", f"{stats.fps:.1f}", c.bullet)

        header("HEALTH")
        for robot in robots:
            health = robot.health
            kv(robot.name, f"{health.hp:.0f}/{health.max_hp:.0f}",
               c.bullet if health.alive else c.hit)
        if outpost is not None:
            kv("Outpost", f"{outpost.health.hp:.0f}/{outpost.health.max_hp:.0f}",
               c.bullet if outpost.health.alive else c.hit)
        if rune is not None:
            mode = "大符" if rune.is_big_rune else "小符"
            state_text = {"idle": "未激活", "activating": "正在激活",
                          "activated": "已激活"}.get(rune.state, str(rune.state))
            kv("Rune", f"{mode} {state_text}",
               c.rune_activated if rune.state == "activated" else c.armor)
            kv("总开关", "开" if rune.armed else "关",
               c.observation if rune.armed else c.robot_dead)
            progress = (f"累计命中 {rune.big_hits} 块" if rune.is_big_rune
                        else f"第 {rune.group_index}/{rune.cfg.groups} 组")
            kv("进度", progress)
            if rune.lit:
                kv("点亮灯臂", " ".join(str(i) for i in rune.lit), c.observation)
            if rune.last_event:
                kv("事件", rune.last_event, _EVENT_FG)

        if draw_texts:
            header("DEBUG")
            for dt in draw_texts:
                text(dt.text, tuple(dt.color))

        return rows

    # ---------------------------------------------------------------- 排版
    def _layout(self, rows, rect):
        """字号 / 列数 / 折行自适应。"""
        font_size = 20
        while True:
            font = self.resources.info_font(font_size)
            line_height = font.get_linesize() + 3
            title_height = line_height

            label_width = 0
            for kind, label, _value, _color in rows:
                if kind == "kv":
                    label_width = max(label_width, font.size(label)[0])
            label_width = min(label_width + 10, 140)

            capacity = max(int((rect.height - title_height - 14) // line_height), 1)
            col_count = 1
            lines_per_row = []
            total = 0
            for col_count in (1, 2):
                col_width = (rect.width - 20) // col_count
                lines_per_row = []
                for kind, label, value, _color in rows:
                    if kind == "kv":
                        lines_per_row.append(
                            self._wrap(value, font, col_width - label_width - 16))
                    elif kind == "header":
                        lines_per_row.append([label])
                    else:
                        lines_per_row.append(self._wrap(value, font, col_width - 14))
                total = sum(len(x) for x in lines_per_row)
                if total <= capacity * col_count:
                    break
            if total <= capacity * col_count or font_size <= 11:
                break
            font_size -= 2

        col_width = (rect.width - 20) // col_count
        return (font, lines_per_row, col_count, title_height,
                label_width, col_width, line_height)

    @staticmethod
    def _wrap(text, font, limit):
        """按实际字宽折行：中文没有空格，必须逐字断（只调一次 metrics）。"""
        if not text:
            return [""]
        metrics = font.metrics(text)
        lines, line, used = [], "", 0.0
        for char, metric in zip(text, metrics):
            char_w = metric[4] if metric else font.size(char)[0]
            if line and used + char_w > limit:
                lines.append(line)
                line, used = char, char_w
            else:
                line += char
                used += char_w
        lines.append(line)
        return lines

    # ---------------------------------------------------------------- 渲染
    def _draw_title_bar(self, rect, font, title_height, stats):
        vp = self.viewport
        c = self.resources.c
        pg.draw.rect(vp.screen, _TITLE_BG,
                     pg.Rect(rect.left, rect.top, rect.width, title_height))
        vp.screen.blit(font.render("PlugYourRM", True, _TITLE_FG),
                       (rect.left + 10, rect.top + 1))
        fps_text = font.render(f"{stats.fps:.0f} fps", True, c.bullet)
        vp.screen.blit(fps_text, (rect.right - 10 - fps_text.get_width(),
                                  rect.top + 1))

    def _draw_rows(self, rect, font, rows, lines_per_row, col_count,
                   title_height, label_width, col_width, line_height):
        vp = self.viewport
        capacity = max(int((rect.height - title_height - 14) // line_height), 1)
        item = 0
        for (kind, label, value, color), line_list in zip(rows, lines_per_row):
            for j, text in enumerate(line_list):
                col, idx = divmod(item, capacity)
                if col >= col_count:
                    break
                x = rect.left + 10 + col * col_width
                y = rect.top + title_height + 6 + idx * line_height
                item += 1
                if kind == "header":
                    if j == 0:
                        vp.screen.blit(font.render(label, True, color), (x, y))
                        pg.draw.line(vp.screen, _RULE,
                                     (x, y + line_height - 4),
                                     (x + col_width - 14, y + line_height - 4), 1)
                    continue
                if kind == "kv":
                    if j == 0:
                        vp.screen.blit(font.render(label, True, _LABEL_FG), (x, y))
                    vp.screen.blit(font.render(text, True, color),
                                   (x + label_width, y))
                    continue
                vp.screen.blit(font.render(text, True, color), (x, y))