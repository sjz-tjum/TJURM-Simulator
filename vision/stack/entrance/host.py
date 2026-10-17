# entrance/host.py
"""宿主循环：选择 -> 跑一局 -> 返回 -> 再选择。ESC 退出。"""
import pygame as pg

from entrance.selector import select
from entrance.session import Session


def run() -> int:
    import custom
    custom.ensure_registered()

    pg.init()
    try:
        while True:
            choice = select()
            if choice is None:
                return 0
            Session(*choice).run()
    finally:
        pg.quit()