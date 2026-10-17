# sim/managers/control_manager.py
"""电控：收自瞄的 ShotSolution，驱动云台，执行发弹。

定位：**伺服 + 物理层**。它不判"该不该打"（归自瞄），只判"物理上能不能现在打"：
    - 用户总开关 auto_aiming（KP9）与 auto_fire（KP8）
    - 冷却（自瞄建议的 solution.cooldown > 0 则用；否则用电控默认）
    - 单发模式下的"新一发"判据：
        * fire_key 非 None：看 fire_key 是否变化。
        * fire_key 为 None（车虚拟点）：看 fire 的上升沿。

**不做弹道解算、不做目标选择、不做跟踪误差检查、不订阅 Prediction。**
"""
from __future__ import annotations

from config import cfg
from core.bus import sys_bus
from core.bus.messages import SysMessages


class ControlManager:
    def __init__(self, robot_manager, bullet_manager):
        self.robot_manager = robot_manager
        self.bullet_manager = bullet_manager

        a = cfg.sim.aim
        self.cooldown_default = float(a.cooldown)

        self._last_fire_time = None
        self._last_fire_key = None
        self._prev_fire = False

    # ---------------------------------------------------------- 主循环
    def update(self, t=None, solution=None):
        robot = self.robot_manager.controlled_robot
        if robot is None:
            return

        gimbal = robot.gimbal
        if not gimbal.auto_aiming:
            self._publish_fire(False, reason="auto_aiming_off")
            self._prev_fire = False
            return

        if solution is None:
            self._hold(gimbal)
            self._publish_fire(False, reason="no_solution")
            self._prev_fire = False
            return

        # ---- 伺服驱动 ----
        gimbal.set_target(
            float(solution.yaw), float(solution.yaw_rate), 0.0,
            -float(solution.pitch), -float(solution.pitch_rate), 0.0,
        )

        # ---- 用户许可（KP8）----
        if not gimbal.auto_fire:
            self._publish_fire(False, reason="auto_fire_off")
            return

        # ---- 自瞄推荐 ----
        if not solution.fire:
            self._prev_fire = False
            self._publish_fire(False, reason="aim_off")
            return

        # 到这里 solution.fire=True

        # ---- 单发模式：判"新一发" ----
        if solution.fire_mode == "single":
            if solution.fire_key is not None:
                if solution.fire_key == self._last_fire_key:
                    self._publish_fire(False, reason="same_target")
                    return
            else:
                if self._prev_fire:
                    self._publish_fire(False, reason="not_rising_edge")
                    return

        # ---- 冷却：自瞄建议优先，否则用默认 ----
        cooldown = (float(solution.cooldown) if solution.cooldown > 0.0
                    else self.cooldown_default)
        if (t is not None and self._last_fire_time is not None
                and float(t) - self._last_fire_time < cooldown):
            self._publish_fire(False, reason="cooldown")
            return

        # ---- 发弹 ----
        self._prev_fire = True
        if t is not None:
            self._last_fire_time = float(t)
            self._last_fire_key = solution.fire_key
        self._publish_fire(
            True, fire_mode=solution.fire_mode,
            fire_key=solution.fire_key, cooldown=cooldown)

    # ---------------------------------------------------------- 输出
    def _hold(self, gimbal):
        gimbal.set_target(float(gimbal.world_rpy[2]), 0.0, 0.0,
                          float(gimbal.world_rpy[1]), 0.0, 0.0)

    def _publish_fire(self, is_fire, **fields):
        sys_bus().publish(SysMessages.FIRE, {"is_fire": bool(is_fire), **fields})

    # ---------------------------------------------------------- 复位
    def reset(self):
        self._last_fire_time = None
        self._last_fire_key = None
        self._prev_fire = False