# core/entities/rigids/rune.py
"""能量机关：5 个灯臂绕 x 轴旋转，可被观测/跟踪/击打。

状态机：idle -> activating -> activated -> (armed 则) activating ...
armed 是总开关：由 trigger 打开、由 cancel 关闭；只有 armed 时才继续下一轮。
随机数一律走外部传入的 numpy Generator（set_rng / rng 参数），不用全局 random。
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from config import cfg
from core.algorithms.math.angle import limit_rad
from core.entities.property.entity_types import EntityType
from core.entities.property.rigid import Rigid
from core.entities.rigids.blade import Blade


class Rune(Rigid):
    def __init__(self, world_pos: Optional[np.ndarray] = None, rng=None, **kwargs):
        c = cfg.rune
        self.cfg = c
        default_pos = np.array([0.0, 0.0, float(c.height)])
        if world_pos is None:
            world_pos = default_pos.copy()
        super().__init__(world_pos=world_pos, **kwargs)
        self._rng = rng if rng is not None else np.random.default_rng()

        self.entity_type = EntityType.Rune
        self.leaf_count = int(c.leaf_count)
        self.radius = float(c.radius)
        self.rotate_dir = int(self._rng.choice([-1, 1]))

        self.is_big_rune = False
        self.state = "idle"
        self.activation_count = 0
        self.armed = False
        self.group_index = 0
        self.lit = []
        self.last_hit = None
        self.marked = []
        self.second_phase = False
        self.window_end = 0.0
        self.big_hits = 0

        self.blade_ring = [None] * self.leaf_count
        self.hit_log = []
        self.activatable = [False] * self.leaf_count
        self.flash_start = None
        self.flash_period = float(c.flash_period)
        self.flash_count = int(c.flash_count)

        self.activation_params = None
        self.activation_timer = 0.0
        self.world_t = 0.0
        self.last_event = ""

        self.blades = []
        self._create_blades()
        self.update_children()

    def set_rng(self, rng) -> None:
        """换一个随机数发生器（仿真器统一种子用）。"""
        self._rng = rng

    # ---------------------------------------------------------- 灯臂
    def _create_blades(self) -> None:
        for i in range(self.leaf_count):
            self.blades.append(Blade(blade_id=i))

    def blade_angle(self, i: int) -> float:
        return self.world_rpy[0] + i * 2 * math.pi / self.leaf_count

    def update_children(self) -> None:
        """把轮面状态传给五个灯臂。"""
        for i, blade in enumerate(self.blades):
            phi = self.blade_angle(i)
            radial = np.array([0, math.cos(phi), math.sin(phi)])
            tangent = np.array([0, -math.sin(phi), math.cos(phi)])
            facing = np.array([-1.0, 0, 0])
            R = np.column_stack([facing, tangent, radial])
            blade.world_pos = self.world_pos + self.radius * radial
            blade.world_R = R
            blade.world_rpy = np.array([phi, 0.0, 0.0])
            blade.world_vel = self.world_vel + np.cross(
                self.world_omg, blade.world_pos - self.world_pos)
            blade.world_omg = self.world_omg.copy()
            for j, pt in enumerate(blade.init_light_corners):
                blade.light_corners[j] = blade.world_pos + R @ pt

    # ---------------------------------------------------------- 阶段
    def set_mode(self, is_big: bool) -> None:
        """切换大小能量机关；切换即取消当前激活。"""
        self.is_big_rune = bool(is_big)
        self.cancel()
        self.last_event = "切到大能量机关" if self.is_big_rune else "切到小能量机关"

    def trigger(self, t: float) -> bool:
        """点亮总开关并开始激活；已在激活中则返回 False。"""
        if self.armed:
            return False
        self.armed = True
        self.state = "activating"
        self.activation_count = 0
        if self.is_big_rune:
            a = float(self._rng.uniform(*cfg.rune.activation_a))
            omg = float(self._rng.uniform(*cfg.rune.activation_omega))
            self.activation_params = {"a": a, "omg": omg,
                                      "b": float(cfg.rune.activation_b_base) - a}
            self.activation_timer = 0.0
        self.group_index = 0
        self.big_hits = 0
        self.blade_ring = [None] * self.leaf_count
        self.hit_log = []
        self.marked = []
        self.flash_start = None
        self._start_group(t)
        self.last_event = "开始激活"
        return True

    def _start_group(self, t: float) -> None:
        if self.is_big_rune:
            pool = [i for i in range(self.leaf_count) if i != self.last_hit]
            if len(pool) < 2:
                pool = list(range(self.leaf_count))
            picked = self._rng.choice(pool, size=2, replace=False)
            self.lit = [int(x) for x in picked]
        else:
            pool = [i for i in range(self.leaf_count) if i not in self.marked]
            if not pool:
                self.marked = []
                pool = list(range(self.leaf_count))
            self.lit = [int(self._rng.choice(pool))]
        self.activatable = [i in self.lit for i in range(self.leaf_count)]
        self.second_phase = False
        self.window_end = t + float(cfg.rune.hit_window)

    def _fail(self, reason: str, t: float) -> None:
        self.last_event = f"{reason}：进度重置，标识随机切换"
        self.group_index = 0
        self.blade_ring = [None] * self.leaf_count
        self.hit_log = []
        self.marked = []
        if self.armed:
            self.state = "activating"
            self._start_group(t)
        else:
            self._reset_activation()

    def _reset_activation(self) -> None:
        self.state = "idle"
        self.lit = []
        self.activatable = [False] * self.leaf_count
        self.second_phase = False
        self.group_index = 0
        self.blade_ring = [None] * self.leaf_count
        self.hit_log = []
        self.marked = []
        self.flash_start = None
        self.activation_params = None

    def cancel(self) -> None:
        """关掉总开关并清空进度。"""
        self.last_event = "已取消激活"
        self.armed = False
        self._reset_activation()

    def reset(self) -> None:
        self.cancel()
        self.activation_count = 0
        self.big_hits = 0
        self.world_t = 0.0
        self.last_event = ""

    # ---------------------------------------------------------- 显示
    def display_rings(self) -> dict:
        """当前要显示的标识 -> 环数映射；闪烁的暗半周返回空。"""
        if self.flash_start is not None:
            half = self.flash_period / 2.0
            if int((self.world_t - self.flash_start) / half) % 2 == 1:
                return {}
            return {b: r for b, r in self.hit_log}
        return {i: r for i, r in enumerate(self.blade_ring) if r is not None}

    def hit_rings(self, blade_id: int) -> list:
        r = self.display_rings().get(blade_id)
        return [] if r is None else [r]

    def _finish_group(self, t: float) -> None:
        if self.is_big_rune:
            self.blade_ring = [None] * self.leaf_count
            self.hit_log = []
            self._start_group(t)
            return
        self.group_index += 1
        if self.group_index >= int(cfg.rune.groups):
            self._activate(t)
        else:
            self._start_group(t)

    def _activate(self, t: float) -> None:
        self.state = "activated"
        self.activation_count += 1
        self.activatable = [False] * self.leaf_count
        self.lit = []
        self.flash_start = t
        self.last_event = "能量机关已激活"

    # ---------------------------------------------------------- 击打
    def ring_of_radius(self, radius: float) -> int:
        r_in = float(cfg.rune.ring_inner_diameter) / 2.0
        r_out = float(cfg.rune.target_diameter) / 2.0
        step = (r_out - r_in) / max(int(cfg.rune.ring_count) - 1, 1)
        k = int((r_out - max(float(radius), 0.0)) / step)
        return int(min(int(cfg.rune.ring_count), max(1, 1 + k)))

    def effective_ring_min(self) -> int:
        table = cfg.rune.ring_min_after_activation
        idx = min(self.activation_count, len(table) - 1)
        return int(table[idx])

    def register_hit(self, blade_id: int, damage: float = 0.0, t: float = 0.0,
                     hit_radius=None, speed=None, caliber: str = "17mm") -> dict:
        if caliber != "17mm" or (speed is not None
                                 and speed < float(cfg.rune.min_bullet_speed)):
            return {"accepted": False, "reason": "受击要求：仅 17mm 且 >12 m/s"}

        ring = self.ring_of_radius(hit_radius) if hit_radius is not None else 1
        # 单一真相：BulletManager._targets 已经过滤掉"非激活中"和"非点亮灯臂"，
        # 所以下面两条守卫在正常路径下不可达，只是本公开 API 的防御性检查。
        # state == "activating" 是"灯臂能吸收子弹"的判据，名字不可改。
        if self.state != "activating":
            return {"accepted": False, "reason": "当前未在激活中", "ring": ring}
        if t > self.window_end:
            self._fail("超出窗口", t)
            return {"accepted": False, "reason": "超时", "ring": ring}
        if blade_id not in self.lit:
            self._fail("击中非点亮灯臂", t)
            return {"accepted": False, "reason": "击中非点亮灯臂", "ring": ring}
        if bool(cfg.rune.ring_gate) and ring < self.effective_ring_min():
            return {"accepted": False, "reason": "环数不足", "ring": ring}

        if self.blade_ring[blade_id] is None:
            self.blade_ring[blade_id] = ring
        self.hit_log.append((blade_id, ring))
        self.last_hit = blade_id
        if not self.is_big_rune:
            self.marked.append(blade_id)
        self.lit.remove(blade_id)
        self.activatable[blade_id] = False
        if self.is_big_rune:
            self.big_hits += 1

        if self.is_big_rune and not self.second_phase and self.lit:
            self.second_phase = True
            self.window_end = t + float(cfg.rune.second_window)
        else:
            self._finish_group(t)
        return {"accepted": True, "ring": ring, "state": self.state,
                "group": self.group_index, "activated": self.state == "activated"}

    # ---------------------------------------------------------- 转速
    def _speed_now(self) -> float:
        if (self.is_big_rune and self.state == "activating"
                and self.activation_params):
            p = self.activation_params
            return p["a"] * math.sin(p["omg"] * self.activation_timer) + p["b"]
        return float(cfg.rune.small_speed if not self.is_big_rune
                     else cfg.rune.big_speed)

    # ---------------------------------------------------------- 运动学步进
    def update(self, dt: float, t: float) -> None:
        """状态机 + 相位积分 + 灯臂更新。t 是仿真时刻（秒）。"""
        t = float(t)
        self.world_t = t

        if self.state == "activating":
            if self.activation_params is not None:
                self.activation_timer += dt
            if t > self.window_end:
                if self.is_big_rune and self.second_phase:
                    self._fail("第二块 1 s 内未击中", t)
                else:
                    self._fail("窗口内未击中点亮灯臂", t)
        elif self.state == "activated":
            # 闪烁满 flash_period * flash_count 秒后收尾
            if (self.flash_start is not None
                    and t - self.flash_start >= self.flash_period * self.flash_count):
                self.blade_ring = [None] * self.leaf_count
                self.hit_log = []
                self.marked = []
                self.flash_start = None
                if self.armed:
                    self.state = "activating"
                    self.group_index = 0
                    self._start_group(t)
                    self.last_event = "继续下一轮"
                else:
                    self._reset_activation()

        self.world_omg[0] = self.rotate_dir * self._speed_now()
        self.world_rpy[0] += self.world_omg[0] * dt
        self.world_rpy[0] = limit_rad(self.world_rpy[0])
        self.update_children()
