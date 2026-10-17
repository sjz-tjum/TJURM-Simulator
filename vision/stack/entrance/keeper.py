# entrance/keeper.py
"""keeper：custom 循环的节拍器。

它**不碰数据**。sim 侧通过 SimComm 发检测/自车状态、收控制指令；custom 侧通过
CustomComm 反向。两边数据交换全部走 public_bus，由各自的 comm 模块负责。

keeper 只做一件事：按 target_fps 叫 pipeline.tick()。它不订阅 sys_bus 的检测，
不管理队列，不发 public_bus。
"""
from __future__ import annotations

import sys
import threading
import time
import traceback
from typing import Optional

from config import cfg
from core.bus import sys_bus
from core.bus.messages import SysMessages


class Keeper:
    def __init__(self, tick=None, reset=None):
        """tick() / reset()：custom 的入口，由装配层注入。两个都无参。"""
        self._tick = tick
        self._reset_custom = reset

        c = cfg.sim.custom
        self.target_fps = float(c.target_fps)

        self._thread: Optional[threading.Thread] = None
        self._stop = False
        self._death_reported = False

        self.stats = {"errors": 0}

        # 只订 reset：sim 复位后通知 custom
        sys_bus().subscribe(SysMessages.RESET, self._on_reset)

    def _on_reset(self, _payload=None):
        if self._reset_custom is not None:
            self._reset_custom()

    # ------------------------------------------------------------ 线程
    def run(self):
        """启动设备线程；已在跑就直接返回它。"""
        if self._thread is not None and self._thread.is_alive():
            return self._thread
        self._stop = False
        self._death_reported = False
        self._thread = threading.Thread(target=self._device_thread_func,
                                        name="custom", daemon=True)
        self._thread.start()
        return self._thread

    def _device_thread_func(self) -> None:
        dt_target = 1.0 / self.target_fps
        while not self._stop:
            start = time.perf_counter()
            try:
                if self._tick is not None:
                    self._tick()
            except Exception:
                self.stats["errors"] += 1
                self._report_death()
                break
            sleep_t = dt_target - (time.perf_counter() - start)
            if sleep_t > 0.0:
                time.sleep(sleep_t)

    def _report_death(self) -> None:
        """设备线程因异常退出：必须让人知道。

        否则现象是"画面还在动、瞄不动了、也不报错"——最难查的一类故障。
        """
        if self._death_reported:
            return
        self._death_reported = True
        tb = traceback.format_exc()
        print("[Keeper] 设备线程因异常退出，解算与跟踪已停止", tb,
              file=sys.stderr)

    def thread_shut_down(self, timeout: float = 5.0) -> None:
        self._stop = True
        thread, self._thread = self._thread, None
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)

    def __del__(self):
        self._stop = True