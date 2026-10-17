# sim/managers/keyboard_manager.py
"""键盘管理器：把 pygame 键盘事件翻译成意图与语义动作。

两类产出：
    持续状态 —— 当前按住了哪些键，`apply_intents()` 每帧合成一次运动意图
    一次性动作 —— 切换 / 创建 / 删除 / 触发，事件到达即执行

职责边界：
    不直接改实体状态（急停走 MotionManager.instant_stop）；不做开火决策；不绘制；
    不管进程退出（`QUIT` 与 `ESC` 归 Runner）。

两条既定约定：
    1. **平移以 W 为参照**（场地轴向）：合成方向后归一化到 TAR_SPEED，与车头朝向无关。
       归一化归本模块 —— MotionManager 只把 tar_vel 当绝对目标速度消费。
    2. **旋转意图与平移意图同形**（都是往 6 维目标里加三维向量），yaw / pitch 各写各的轴。
       把旋转压成一个标量会把俯仰塞进进偏航通道，这个错误在形状上被杜绝。

开火开关的分工：KP9 = 自动瞄准、KP8 = 自动开火（都在 gimbal，作用于受控车）、
KP0 = 手动连发（在 BulletManager，不受 auto_fire 约束，用于不开自瞄试弹道）。
"""
from __future__ import annotations

from typing import Optional, Set

import numpy as np

from core.entities.property.entity_types import EntityType
from core.entities.property.motion import Motion, MotionConfig
import pygame as pg


class KeyboardManager:
    def __init__(self, simulator):
        self.simulator = simulator
        self.pressed_keys: Set[int] = set()

        # ------------------------------------------------ 键位表（数据，不是逻辑）
        # 平移：键 -> 场地轴单位向量（+x 俯视图向上、+y 向左、+z 向上）
        self.trans_keys = {
            pg.K_UP: Motion.FORWARD, pg.K_DOWN: Motion.BACKWARD,
            pg.K_LEFT: Motion.LEFT, pg.K_RIGHT: Motion.RIGHT,
            pg.K_g: Motion.UP, pg.K_b: Motion.DOWN,
        }

        def about_z(value):
            return np.array([0.0, 0.0, float(value)])

        def about_y(value):
            return np.array([0.0, float(value), 0.0])

        # 底盘旋转：绕 Z
        self.chassis_rot_keys = {
            pg.K_a: about_z(Motion.CHASSIS_CCW_SLOW),
            pg.K_d: about_z(Motion.CHASSIS_CW_SLOW),
            pg.K_z: about_z(Motion.CHASSIS_CCW_FAST),
            pg.K_c: about_z(Motion.CHASSIS_CW_FAST),
        }
        # 云台旋转：Q/E 绕 Z（偏航），W/X 绕 Y（俯仰）
        self.gimbal_rot_keys = {
            pg.K_q: about_z(Motion.GIMBAL_YAW_LEFT),
            pg.K_e: about_z(Motion.GIMBAL_YAW_RIGHT),
            pg.K_w: about_y(Motion.GIMBAL_PITCH_UP),
            pg.K_x: about_y(Motion.GIMBAL_PITCH_DOWN),
        }

        # 组合键目标：Enter + 数字建车、Backspace + 数字删车
        self.create_keys = {
            pg.K_1: EntityType.Hero,
            pg.K_2: EntityType.Infantry,
            pg.K_3: EntityType.Sentry,
            pg.K_4: EntityType.Engineer,
        }
        self.digit_keys = {getattr(pg, "K_%d" % i): i for i in range(10)}

        self.spec_key_func_map = {
            pg.K_r: self._reset,
            pg.K_SPACE: self._instant_stop,
            pg.K_KP9: self._toggle_auto_aiming,
            pg.K_KP8: self._toggle_auto_fire,
            pg.K_KP0: self._toggle_manual_trigger,
            pg.K_KP7: self._cycle_preset,
            pg.K_m: self._toggle_curve_panel,
            pg.K_KP4: self._cycle_curve_preset,
            pg.K_KP5: self._switch_rune_mode,
            pg.K_KP6: self._trigger_rune,
        }

    # ------------------------------------------------------------ 事件
    def handle_event(self, event) -> None:
        """每个 KEYDOWN / KEYUP / 窗口事件调一次。退出与 QUIT 归 Runner，这里不管。"""
        if event.type == pg.WINDOWFOCUSLOST:
            # 失焦时操作系统不发 KEYUP：残留的按键会持续注入意图（车一直走）。
            # 清空 pressed_keys，下一帧起 apply_intents 不再产生任何意图。
            self.pressed_keys.clear()
            return
        if event.type == pg.KEYDOWN:
            self.pressed_keys.add(event.key)
            if not self.handle_combo_key(event.key):
                self.handle_single_key(event.key)
        elif event.type == pg.KEYUP:
            self.pressed_keys.discard(event.key)

    # ------------------------------------------------------------ 意图
    def apply_intents(self) -> None:
        """把当前按住的键合成成意图注入 MotionManager。每帧调一次（在 run 之前）。"""
        robot = self.simulator.robot_manager.controlled_robot
        if robot is None:
            return
        chassis = robot.chassis
        gimbal = robot.gimbal
        mm = self.simulator.motion_manager

        # 平移：多键合成方向，归一化到 TAR_SPEED
        total = np.zeros(3)
        for key, vec in self.trans_keys.items():
            if key in self.pressed_keys:
                total += vec
        norm = float(np.linalg.norm(total))
        if norm > 1e-9:
            mm.add_intents(chassis, [self._make_intent(total / norm * MotionConfig.TAR_SPEED)])

        # 底盘旋转：各键叠加，不归一化（同时按慢转+快转就该更快）
        omg = np.zeros(3)
        for key, vec in self.chassis_rot_keys.items():
            if key in self.pressed_keys:
                omg += vec
        if float(np.linalg.norm(omg)) > 1e-9:
            mm.add_intents(chassis, [self._make_intent(omg=omg)])

        # 云台旋转：自瞄接管时不叠加手动角速度
        if not gimbal.auto_aiming:
            omg = np.zeros(3)
            for key, vec in self.gimbal_rot_keys.items():
                if key in self.pressed_keys:
                    omg += vec
            if float(np.linalg.norm(omg)) > 1e-9:
                mm.add_intents(gimbal, [self._make_intent(omg=omg)])

    @staticmethod
    def _make_intent(vel=None, omg=None):
        """一个意图同时携带平移与旋转分量，两者都是三维向量。"""
        tar_vel = np.zeros(3) if vel is None else np.asarray(vel, float)
        tar_omg = np.zeros(3) if omg is None else np.asarray(omg, float)

        def intent(snapshot, t, dt):
            nonlocal tar_vel, tar_omg
            snapshot.tar_vel += tar_vel
            snapshot.tar_omg += tar_omg

        return intent

    # ------------------------------------------------------------ 组合键
    def handle_combo_key(self, key: int) -> bool:
        """返回是否已被组合键消费。"""
        if pg.K_BACKSPACE in self.pressed_keys:
            if key in self.digit_keys:
                self.simulator.robot_manager.remove_robot(self.digit_keys[key])
                return True
            if key == pg.K_KP1:
                self.simulator.device_manager.remove_device(EntityType.Outpost)
                return True
            if key == pg.K_KP2:
                self.simulator.device_manager.remove_device(EntityType.Rune)
                return True

        if pg.K_RETURN in self.pressed_keys:
            if key in self.create_keys:
                self.simulator.robot_manager.add_robot(self.create_keys[key])
                return True
            if key == pg.K_KP1:
                self.simulator.device_manager.add_device(EntityType.Outpost)
                return True
            if key == pg.K_KP2:
                self.simulator.device_manager.add_device(EntityType.Rune)
                return True

        return False

    # ------------------------------------------------------------ 单键
    def handle_single_key(self, key: int) -> None:
        if key == pg.K_1:
            self.simulator.robot_manager.switch_control_robot()
            return
        func = self.spec_key_func_map.get(key)
        if func is not None:
            func()

    # ------------------------------------------------------------ 特殊动作
    def _reset(self) -> None:
        """R：软重置（由 Simulator 统一调度各管理器）。"""
        self.simulator.reset()

    def _instant_stop(self) -> None:
        """Space：急停底盘。云台不受影响 —— 自瞄或手动接管继续驱动它。"""
        robot = self.simulator.robot_manager.controlled_robot
        if robot is not None:
            self.simulator.motion_manager.instant_stop(robot.chassis)

    def _toggle_auto_aiming(self) -> None:
        robot = self.simulator.robot_manager.controlled_robot
        if robot is not None:
            robot.gimbal.switch_auto_aiming()

    def _toggle_auto_fire(self) -> None:
        robot = self.simulator.robot_manager.controlled_robot
        if robot is not None:
            robot.gimbal.switch_auto_fire()

    def _toggle_manual_trigger(self) -> None:
        self.simulator.bullet_manager.switch_manual_trigger()

    def _cycle_preset(self) -> None:
        """KP7：目标机动预设循环（只作用于非受控底盘）。"""
        self.simulator.motion_manager.set_preset()

    def _toggle_curve_panel(self) -> None:
        self.simulator.render_manager.switch_curve_panel()

    def _cycle_curve_preset(self) -> None:
        self.simulator.render_manager.cycle_curve_preset()

    def _switch_rune_mode(self) -> None:
        self.simulator.device_manager.switch_rune_mode()

    def _trigger_rune(self) -> None:
        self.simulator.device_manager.trigger_rune()