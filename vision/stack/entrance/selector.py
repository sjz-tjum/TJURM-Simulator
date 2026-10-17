# entrance/selector.py
"""启动选择界面：两列（Tracker / PoseSolver），逐列确认。

每列选一个实现，两列都确认后返回 (tracker_name, pose_solver_name)；ESC 退出。
pygame 生命周期由 host 持有，这里只 set_mode —— 尺寸和游戏一致。
"""
from __future__ import annotations

import pygame as pg

from config import cfg
from core.interfaces.pose_solver import PoseSolver
from core.interfaces.tracker import Tracker

WIDTH = cfg.ui.screen_width
HEIGHT = cfg.ui.screen_height

BG = (22, 24, 28)
FG = (222, 228, 236)
DIM = (120, 128, 140)
ACCENT = (255, 200, 0)
ROW_BG = (52, 60, 74)
RULE = (70, 76, 86)
CONFIRMED = (110, 220, 120)

_CJK = ("microsoftyahei", "simhei", "simsun", "dengxian")
COLUMNS = (("Tracker", Tracker), ("PoseSolver", PoseSolver))

_MARGIN = 100
_GAP = 60
_TOP = 200
_ROW_H = 72


def _load_font(size: int):
    for name in _CJK:
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


def select():
    """显示选择界面，返回 (tracker_name, pose_solver_name)；ESC / 关窗口返回 None。"""
    screen = pg.display.set_mode((WIDTH, HEIGHT))
    pg.display.set_caption("PlugYourRM — 选择实现")

    items = [(title, reg.items()) for title, reg in COLUMNS]
    cursor = [0, 0]
    active = 0
    confirmed = [False, False]

    title_font = _load_font(38)
    header_font = _load_font(26)
    item_font = _load_font(22)
    desc_font = _load_font(16)
    hint_font = _load_font(18)
    clock = pg.time.Clock()

    while True:
        for e in pg.event.get():
            if e.type == pg.QUIT:
                return None
            if e.type != pg.KEYDOWN:
                continue
            if e.key == pg.K_ESCAPE:
                return None
            if e.key in (pg.K_RETURN, pg.K_KP_ENTER):
                confirmed[active] = True
                rest = [i for i in range(2) if not confirmed[i]]
                if not rest:
                    return tuple(items[i][1][cursor[i]][0] for i in range(2))
                active = rest[0]
            elif e.key in (pg.K_LEFT, pg.K_a):
                active = (active - 1) % 2
            elif e.key in (pg.K_RIGHT, pg.K_d, pg.K_TAB):
                active = (active + 1) % 2
            elif e.key in (pg.K_UP, pg.K_w):
                n = len(items[active][1])
                cursor[active] = (cursor[active] - 1) % n
                confirmed[active] = False
            elif e.key in (pg.K_DOWN, pg.K_s):
                n = len(items[active][1])
                cursor[active] = (cursor[active] + 1) % n
                confirmed[active] = False

        _draw(screen, title_font, header_font, item_font, desc_font,
              hint_font, items, cursor, active, confirmed)
        pg.display.flip()
        clock.tick(60)


def _draw(screen, title_font, header_font, item_font, desc_font, hint_font,
          items, cursor, active, confirmed):
    screen.fill(BG)

    title = title_font.render("PlugYourRM", True, FG)
    screen.blit(title, ((WIDTH - title.get_width()) // 2, 46))
    sub = hint_font.render("选择 Tracker 和 PoseSolver", True, DIM)
    screen.blit(sub, ((WIDTH - sub.get_width()) // 2, 104))
    hint = hint_font.render(
        "↑↓ 选项    ←→ 切列    Enter 确认 / 前进    ESC 退出", True, DIM)
    screen.blit(hint, ((WIDTH - hint.get_width()) // 2, 134))

    col_w = (WIDTH - 2 * _MARGIN - _GAP) // 2
    for i, (title_text, entries) in enumerate(items):
        x = _MARGIN + i * (col_w + _GAP)
        is_active = (i == active)
        is_confirmed = confirmed[i]

        if is_confirmed:
            header_color = CONFIRMED
            display = f"√ {title_text}"
        elif is_active:
            header_color = ACCENT
            display = title_text
        else:
            header_color = FG
            display = title_text

        screen.blit(header_font.render(display, True, header_color), (x, _TOP))
        pg.draw.line(screen, RULE, (x, _TOP + 38), (x + col_w, _TOP + 38), 1)

        for j, (name, desc) in enumerate(entries):
            y = _TOP + 60 + j * _ROW_H
            selected = (j == cursor[i])
            rect = pg.Rect(x - 12, y - 8, col_w + 24, _ROW_H - 8)
            if selected:
                pg.draw.rect(screen, ROW_BG, rect, border_radius=6)
                if is_active:
                    pg.draw.rect(screen, ACCENT, rect, 2, border_radius=6)
                elif is_confirmed:
                    pg.draw.rect(screen, CONFIRMED, rect, 2, border_radius=6)

            if selected and is_active:
                name_color = ACCENT
            elif selected and is_confirmed:
                name_color = CONFIRMED
            else:
                name_color = FG
            screen.blit(item_font.render(name, True, name_color), (x, y))
            screen.blit(desc_font.render(_clip(desc, 40), True, DIM),
                        (x, y + 30))


def _clip(text: str, limit: int) -> str:
    return text if len(text) <= limit else text[:limit - 1] + "…"