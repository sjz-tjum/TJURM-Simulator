# sim/managers/bullet_manager.py
"""弹丸生命周期：从扣扳机到扣血的完整链路。

职责：
    受理开火请求（订阅 sys_bus 的 fire）-> 延时发射 -> 推进飞行
    -> 检测命中 -> 结算（扣血 / 记环）。

不负责：
    开火决策（ControlManager）、能量机关的规则节奏（Rune 自己管窗口与环数）、
    弹道解算（ControlManager）、绘制（RenderManager 只读 .bullets）。

对外被依赖：
    .caliber       当前弹种，ControlManager 用它选弹道参数
    .bullets       弹丸列表，RenderManager 读它画弹丸与命中点
    .pull_trigger()  手动开火

约定：
    弹种当前固定 17mm（cfg.bullet 里另有 42mm，本仿真未启用）。
    弹种随车变化留到真正需要时再做，现在不做半成品。

时序：
    Simulator._advance 的顺序是 motion -> aim -> bullet -> sensor。
    control_manager.update(t) 内部会发布 fire，因此 _on_fire 在 bullet_manager.update
    之前被调用，拿不到本帧的 t——它读 self._t（上一帧缓存）。这是本类唯一的时间缓存，
    误差一帧（200 fps 下 5 ms），对 fire_delay 无实际影响。
"""
from __future__ import annotations

import numpy as np

from config import cfg
from core.algorithms.math.intersect import segment_disc_hit, segment_rect_hit
from core.algorithms.math.transform import rpy_to_R
from core.bus import public_bus, sys_bus
from core.bus.dataflow import DrawText
from core.bus.messages import PublicMessages, SysMessages
from core.entities.rigids.bullet import Bullet
from core.entities.property.entity_types import EntityType


class BulletManager:
    def __init__(self, robot_manager, device_manager):
        self.robot_manager = robot_manager
        self.device_manager = device_manager

        self.bullets = []
        self.pending_fires = []

        c = cfg.sim.bullet
        self.fire_delay = float(c.fire_delay)
        self.manual_interval = float(c.manual_interval)
        self.max_age = float(c.max_age)
        self.max_range = float(c.max_range)
        self.hit_retention = float(c.hit_retention)

        self.caliber = "17mm"
        self.manual_trigger = False
        self.last_fire_time = -1e9
        self._t = 0.0

        sys_bus().subscribe(SysMessages.FIRE, self._on_fire)

    # ---------------------------------------------------------- 开火请求
    def _on_fire(self, data):
        """受理开火请求：本车是否允许发弹已由 control_manager 判过，这里不再查。"""
        if not data.get("is_fire"):
            return
        robot = self.robot_manager.controlled_robot
        if robot is None:
            return
        _, muzzle_vel = self._muzzle_state(robot)
        # 只锁方向（速度矢量），位置等真正出膛时再取
        self.pending_fires.append((self._t + self.fire_delay, robot, muzzle_vel))

    def switch_manual_trigger(self):
        self.manual_trigger = not self.manual_trigger

    def pull_trigger(self):
        """按当前枪口朝向立即发一颗（手动开火，不受 auto_fire 约束）。"""
        robot = self.robot_manager.controlled_robot
        if robot is None:
            return None
        self.last_fire_time = self._t
        return self.fire_bullet(*self._muzzle_state(robot), shooter=robot)

    # ---------------------------------------------------------- 发射
    def _muzzle_state(self, robot):
        """返回 (枪口世界位置, 出膛世界速度)。"""
        muzzle = robot.get_muzzle()
        # 枪口朝向 = 云台姿态的 x 轴（FLU：前）。与 Camera.get_forward_vec 同一写法。
        direction = rpy_to_R(robot.gimbal.world_rpy) @ np.array([1.0, 0.0, 0.0])
        # 枪口自身的世界速度（随云台旋转的切向速度）必须算进去——这是全场唯一必须用 W 系处
        speed = float(cfg.bullet._17mm.speed)
        return muzzle.world_pos.copy(), muzzle.world_vel + speed * direction

    def fire_bullet(self, muzzle_pos, velocity, shooter=None):
        spec = cfg.bullet._17mm
        bullet = Bullet(muzzle_pos, velocity,
                        mass=spec.mass, caliber=spec.caliber,
                        drag_cd=spec.drag_cd, damage=spec.damage,
                        shooter=shooter)
        self.bullets.append(bullet)
        return bullet

    # ---------------------------------------------------------- 主循环
    def update(self, dt, t):
        self._t = float(t)
        self._update_triggers(t)
        self._update_bullets(dt, t)

    def _update_triggers(self, t):
        """扳机侧：到点的延时发射 + 手动连发节流。"""
        for item in list(self.pending_fires):
            fire_time, robot, latched_vel = item
            if t < fire_time:
                continue
            self.pending_fires.remove(item)
            if robot not in self.robot_manager.robots:
                continue        # 车已删除，这一发不打了
            muzzle = robot.get_muzzle()
            self.fire_bullet(muzzle.world_pos.copy(), latched_vel.copy(),
                             shooter=robot)

        if self.manual_trigger and t - self.last_fire_time >= self.manual_interval:
            self.pull_trigger()

    def _update_bullets(self, dt, t):
        """世界侧：弹丸步进、淘汰、命中检测。"""
        for bullet in list(self.bullets):
            if bullet.hit:
                if t - bullet.hit_time > self.hit_retention:
                    self.bullets.remove(bullet)
                continue

            prev_pos = bullet.world_pos.copy()
            bullet.update(dt)
            if (bullet.age > self.max_age
                    or float(np.linalg.norm(bullet.world_pos - bullet.origin)) > self.max_range
                    or bullet.world_pos[2] < 0.0):
                self.bullets.remove(bullet)
                continue
            self._resolve_hit(bullet, prev_pos, t)


    # ---------------------------------------------------------- 命中
    def _resolve_hit(self, bullet, prev_pos, t):
        """一颗弹一帧最多结算一块装甲板。"""
        for target, armor in self._targets():
            hit, hit_pos = self._hit_armor(prev_pos, bullet.world_pos, armor)
            if not hit:
                continue
            bullet.hit = True
            bullet.hit_pos = hit_pos
            bullet.hit_target = (target.entity_type, armor.armor_id)
            bullet.hit_time = t
            if target.entity_type == EntityType.Rune:
                self._resolve_rune_hit(bullet, target, armor, hit_pos, t)
            else:
                self._resolve_entity_hit(bullet, target, t)
            return

    def _hit_armor(self, prev_pos, curr_pos, armor):
        """把装甲板拆成基元，交给 core 的纯几何函数判定。"""
        corners = armor.light_corners
        normal = np.cross(corners[1] - corners[0], corners[3] - corners[0])
        # 法线必须归一化：intersect 里的共面容差是绝对量，未归一化会被 |cross| 缩放；
        # 角点共线时 |cross|≈0 会让"处处共面"假命中，所以退化直接判不中。
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            return False, None
        normal = normal / norm
        if str(getattr(armor, "hit_shape", "rect")) == "disc":
            return segment_disc_hit(prev_pos, curr_pos,
                                    np.asarray(armor.world_pos, float), normal,
                                    float(armor.hit_radius))
        return segment_rect_hit(prev_pos, curr_pos, corners[0],
                                corners[1] - corners[0],
                                corners[3] - corners[0], normal)

    def _resolve_rune_hit(self, bullet, rune, armor, hit_pos, t):
        radius = float(np.linalg.norm(hit_pos - armor.world_pos))
        res = rune.register_hit(armor.armor_id, bullet.damage, t,
                                hit_radius=radius,
                                speed=float(np.linalg.norm(bullet.world_vel)),
                                caliber=self.caliber)
        if res.get("accepted"):
            text = f"命中 {res['ring']} 环：{rune.group_index}/{cfg.rune.groups} 组"
            if res.get("activated"):
                text += "（已激活）"
            color = (255, 120, 0)
        else:
            text = str(res.get("reason", "未计入"))
            color = (255, 80, 80)
        public_bus().publish(PublicMessages.DEBUG_TEXT,
                             DrawText(text=text, color=color))

    def _resolve_entity_hit(self, bullet, target, t):
        target.health.take_damage(bullet.damage, t)

    # ---------------------------------------------------------- 目标
    def _targets(self):
        """可吸收子弹的目标枚举（命中策略的接缝）。

        规则（这里的过滤是唯一真相，Rune 侧的同名守卫只是防御性检查）：
            - 不打自己：排除当前 controlled；
            - 不打死人：排除已阵亡的机器人；
            - 能量机关只在激活中、且只有点亮灯臂吸收子弹；非点亮灯臂子弹穿过去，
              不结算、不重置整组进度。
        """
        for robot in self.robot_manager.robots:
            if robot is self.robot_manager.controlled_robot:
                continue
            if not robot.health.alive:
                continue
            for armor in robot.get_armors():
                yield robot, armor

        outpost = self.device_manager.outpost
        if outpost is not None and outpost.health.alive:
            for armor in outpost.armors:
                yield outpost, armor

        rune = self.device_manager.rune
        if rune is not None and rune.state == "activating":
            for blade in rune.blades:
                if blade.armor_id in rune.lit:
                    yield rune, blade

    # ---------------------------------------------------------- 重置
    def reset(self):
        self.bullets.clear()
        self.pending_fires.clear()
        self.manual_trigger = False
        self.last_fire_time = -1e9