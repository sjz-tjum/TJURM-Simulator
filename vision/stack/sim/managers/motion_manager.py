# sim/managers/motion_manager.py
"""运动管理器：对 registry 里的实体执行物理步进或运动学步进。

概念：
    Intent（运动意图）      —— 函数 (snapshot, t, dt) -> None，往 tar_vel/tar_omg 上累加。
    MotionSnapshot（快照）  —— 某一帧某一物理实体的运动状态副本。
    Physics Step（物理步进）—— tar_vel/tar_omg 经加速度、摩擦、限幅变成 vel/pos/rpy。
    Kinematic Step（运动学）—— 直接写世界状态，由设备自己实现 update(dt, t)。

意图叠加：同一实体可被多个来源追加意图（键盘、预设），物理步进时统一消费。
时间参数：t 是绝对时间，dt 是本帧时长；意图自行决定用哪个。

两条关键约定：
    1. 顺序契约：先所有 chassis，再所有 gimbal —— chassis.update_children 把云台位置推好之后，
       云台才能做旋转步进；云台自身不积分平移（位置是父级推的）。
    2. 意图在 update 末尾统一清空，下一帧由 KeyboardManager / 预设重新注入。
       归一化不在这里做（归 KeyboardManager）；这里只把 tar_vel 当绝对目标速度用。

预设（KP7 循环）：static / spin_slow / spin_fast / patrol。
两个 spin 对应自瞄 mode 阈值（MultiTargetParams.spin_thresh = 3.0）的两侧——
慢速走车 auto（追板），快速走车 single（虚拟点）。
"""
from __future__ import annotations

import math
import weakref
from typing import Callable, Iterable

import numpy as np

from config import cfg
from core.algorithms.math.angle import limit_rad
from core.entities.property.motion import MotionSnapshot
from core.entities.rigids.chassis import Chassis

IntentFn = Callable[[MotionSnapshot, float, float], None]


class MotionManager:
    PRESETS = ("static", "spin_slow", "spin_fast", "patrol")

    def __init__(self, registry) -> None:
        self.registry = registry
        self._intents = weakref.WeakKeyDictionary()
        self._controlled_getter = lambda: None

        self._preset = "static"
        self._patrol_state = weakref.WeakKeyDictionary()

        m = cfg.sim.motion
        self.spin_slow = float(m.spin_slow)
        self.spin_fast = float(m.spin_fast)
        self.patrol_radius = float(m.patrol_radius)
        self.patrol_speed = float(m.patrol_speed)
        self.patrol_radial_gain = float(m.patrol_radial_gain)

    # ---------------------------------------------------------- 意图
    def add_intents(self, entity, intents: Iterable[IntentFn]) -> None:
        buf = self._intents.get(entity)
        if buf is None:
            buf = []
            self._intents[entity] = buf
        buf.extend(intents)

    def clear_intents(self, entity) -> None:
        self._intents[entity] = []

    def instant_stop(self, entity) -> None:
        if entity is None:
            return
        entity.world_vel[:] = 0.0
        entity.world_omg[:] = 0.0
        self.clear_intents(entity)

    # ---------------------------------------------------------- 受控查询
    def set_controlled_getter(self, getter) -> None:
        """注入回调而不是持有 RobotManager：受控车不能被预设驱动。"""
        self._controlled_getter = getter

    # ---------------------------------------------------------- 预设
    def set_preset(self, name=None) -> str:
        if name is None:
            idx = self.PRESETS.index(self._preset)
            name = self.PRESETS[(idx + 1) % len(self.PRESETS)]
        if name not in self.PRESETS:
            raise ValueError(f"未知预设 {name!r}；可用：{self.PRESETS}")
        self._preset = name
        self._patrol_state.clear()
        return name

    def _apply_preset(self, entity, snapshot, t, dt) -> None:
        """预设只驱动非受控底盘；云台与独立设备不参与。"""
        if not isinstance(entity, Chassis):
            return
        if self._controlled_getter() is entity:
            return
        name = self._preset
        if name == "static":
            return
        if name == "spin_slow":
            snapshot.tar_omg[2] += self.spin_slow
            return
        if name == "spin_fast":
            snapshot.tar_omg[2] += self.spin_fast
            return
        if name == "patrol":
            state = self._patrol_state.get(entity)
            if state is None:
                state = {
                    "theta": 0.0,
                    "center": np.asarray(snapshot.pos[:2], float).copy(),
                }
                self._patrol_state[entity] = state
            omega = self.patrol_speed / max(self.patrol_radius, 1e-6)
            state["theta"] += omega * dt
            theta = state["theta"]
            cx, cy = state["center"]
            # 切向前馈：目标圆周上该相位处的切向速度
            v_t = self.patrol_speed * np.array([-math.sin(theta), math.cos(theta)])
            # 径向校正：把当前半径拉回目标值
            px, py = snapshot.pos[0], snapshot.pos[1]
            radial = np.array([px - cx, py - cy])
            r_now = float(np.linalg.norm(radial))
            if r_now < 1e-9:
                radial_dir = np.array([math.cos(theta), math.sin(theta)])
            else:
                radial_dir = radial / r_now
            v_r = self.patrol_radial_gain * (self.patrol_radius - r_now) * radial_dir
            snapshot.tar_vel[0] += v_t[0] + v_r[0]
            snapshot.tar_vel[1] += v_t[1] + v_r[1]

    # ---------------------------------------------------------- 主循环
    def update(self, dt: float, t: float) -> None:
        # 顺序契约：必须先所有 chassis，再所有 gimbal。
        # chassis.update_children 把 gimbal 的位置推好，gimbal 才能做旋转步进。
        for chassis in self.registry.chassis:
            self._step_physics(chassis, dt, t)
            chassis.update_children()

        for gimbal in self.registry.gimbals:
            if gimbal.auto_aiming:
                gimbal.apply_control(dt)
            else:
                self._step_gimbal_physics(gimbal, dt, t)
            gimbal.update_children()

        for outpost in self.registry.outposts:
            outpost.update(dt, t)
        for rune in self.registry.runes:
            rune.update(dt, t)

        # 意图消费完清空，下一帧由 KeyboardManager / 预设重新注入
        self._intents.clear()

    # ---------------------------------------------------------- 物理步进
    def _collect_snapshot(self, entity, dt: float, t: float) -> MotionSnapshot:
        """构造状态快照并应用本帧所有意图（预设 + 键盘）。"""
        snapshot = MotionSnapshot(
            pos=entity.world_pos.copy(),
            vel=entity.world_vel.copy(),
            rpy=entity.world_rpy.copy(),
            omg=entity.world_omg.copy(),
        )
        self._apply_preset(entity, snapshot, t, dt)
        for intent in self._intents.get(entity, ()):
            intent(snapshot, t, dt)
        return snapshot

    def _step_physics(self, entity, dt: float, t: float) -> None:
        """平移 + 旋转物理步进，写回位置、速度、朝向、角速度。"""
        snapshot = self._collect_snapshot(entity, dt, t)
        _apply_physics_trans(snapshot, dt)
        _apply_physics_rot(snapshot, dt)
        entity.world_pos = snapshot.pos
        entity.world_vel = snapshot.vel
        entity.world_rpy = snapshot.rpy
        entity.world_omg = snapshot.omg

    def _step_gimbal_physics(self, entity, dt: float, t: float) -> None:
        """只做旋转物理步进；位置由父级 chassis.update_children 推。"""
        snapshot = self._collect_snapshot(entity, dt, t)
        _apply_physics_rot(snapshot, dt)
        entity.world_rpy = snapshot.rpy
        entity.world_omg = snapshot.omg

    # ---------------------------------------------------------- 重置
    def reset(self) -> None:
        self._intents.clear()
        self._preset = "static"
        self._patrol_state.clear()


def _apply_physics_trans(snapshot: MotionSnapshot, dt: float) -> None:
    vel_error = snapshot.tar_vel - snapshot.vel
    control_force = snapshot.motor_gain * vel_error
    fric_force = -snapshot.fric_coef * snapshot.vel
    total_force = control_force + fric_force
    force_norm = float(np.linalg.norm(total_force))
    if force_norm > snapshot.max_force:
        total_force = total_force / force_norm * snapshot.max_force

    snapshot.acc = total_force / snapshot.mass
    snapshot.vel += snapshot.acc * dt
    speed = float(np.linalg.norm(snapshot.vel))
    if speed > snapshot.max_speed:
        snapshot.vel = snapshot.vel / speed * snapshot.max_speed

    snapshot.pos += snapshot.vel * dt


def _apply_physics_rot(snapshot: MotionSnapshot, dt: float) -> None:
    omg_error = snapshot.tar_omg - snapshot.omg
    control_torque = snapshot.motor_gain * omg_error
    fric_torque = -snapshot.fric_coef * snapshot.omg
    snapshot.alp = control_torque + fric_torque
    snapshot.omg += snapshot.alp * dt
    rotate_speed = float(np.linalg.norm(snapshot.omg))
    if rotate_speed > snapshot.max_rotate_speed:
        snapshot.omg = snapshot.omg / rotate_speed * snapshot.max_rotate_speed

    snapshot.rpy += snapshot.omg * dt
    snapshot.rpy[2] = limit_rad(snapshot.rpy[2])
    snapshot.rpy[1] = float(np.clip(snapshot.rpy[1], -np.pi / 2, np.pi / 2))