# entrance/session.py
"""一局游戏：装总线 -> boot custom -> 装配 sim -> 主循环 -> 关停。

每次调用都是全新的：新总线、新 custom、新 sim。pygame 生命周期由 host 持有。
"""
from __future__ import annotations

import sys
import traceback


class Session:
    def __init__(self, tracker_name, pose_solver_name):
        self.tracker_name = tracker_name
        self.pose_solver_name = pose_solver_name

    def run(self) -> int:
        # 每次进游戏都换新总线：旧订阅随旧总线废弃，不会累积。
        from core.bus import EventBus, install_sys_bus, install_public_bus
        install_sys_bus(EventBus())
        install_public_bus(EventBus())

        # 两个通信模块各自订阅各自方向的消息
        from sim.comm import SimComm
        from custom.comm import CustomComm
        sim_comm = SimComm()
        custom_comm = CustomComm()

        import custom
        pipeline = custom.boot(
            comm=custom_comm,
            tracker_name=self.tracker_name,
            pose_solver_name=self.pose_solver_name)
        try:
            return self._loop(pipeline, sim_comm)
        except Exception:
            print("[Session] 本局异常退出：\n" + traceback.format_exc(),
                  file=sys.stderr)
            return 1
        finally:
            custom.shutdown()

    def _loop(self, pipeline, sim_comm) -> int:
        import pygame as pg

        from sim.simulator import Simulator
        from sim.managers.keyboard_manager import KeyboardManager
        from entrance.keeper import Keeper

        pg.display.set_caption(f"PlugYourRM - {self.tracker_name}")

        simulator = Simulator(comm=sim_comm)
        keyboard_manager = KeyboardManager(simulator)
        keeper = Keeper(tick=pipeline.tick, reset=pipeline.reset)
        keeper.run()

        running = True
        try:
            while running:
                for event in pg.event.get():
                    if event.type == pg.QUIT:
                        running = False
                        break
                    if event.type == pg.KEYDOWN and event.key == pg.K_ESCAPE:
                        running = False
                        break
                    keyboard_manager.handle_event(event)
                keyboard_manager.apply_intents()
                simulator.run()
                pg.display.flip()
        finally:
            keeper.thread_shut_down()
        return 0