# sim/managers/robot_manager.py
"""机器人管理器：机器人的账本 + 角色分配。

三层职责：
    生命周期 —— add_robot / remove_robot（同时维护注册表）
    角色     —— controlled_robot 的切换（只有 controlled，没有 viewing）
    状态复位 —— reset：软重置，清状态但不复位选择

不驱动目标机动（归 MotionManager），不猜位置（谁建谁传位置）。
"""
from __future__ import annotations

from core.entities.property.entity_types import EntityType, RegistryType
from core.entities.rigids.robot import Robot


class RobotManager:
    def __init__(self, registry):
        self.registry = registry
        self.robots = []
        self.controlled_robot = None

    # ---------------------------------------------------------- 增删
    def add_robot(self, entity_type, chassis_pos=None, chassis_rpy=None,
                  gimbal_mount_pos=None, gimbal_mount_rpy=None):
        robot = Robot(entity_type, chassis_pos, chassis_rpy,
                      gimbal_mount_pos, gimbal_mount_rpy)
        self.robots.append(robot)
        self.registry.add(robot.chassis, RegistryType.CHASSIS)
        self.registry.add(robot.gimbal, RegistryType.GIMBAL)
        if self.controlled_robot is None:
            self.controlled_robot = robot

    def remove_robot(self, robot_id):
        if not (0 <= robot_id < len(self.robots)):
            print(f"[RobotManager] robot index {robot_id} out of range")
            return
        robot = self.robots[robot_id]
        self.registry.remove(robot.chassis)
        self.registry.remove(robot.gimbal)
        if self.controlled_robot == robot:
            self.controlled_robot = None
        self.robots.pop(robot_id)
        if self.robots and self.controlled_robot is None:
            self.controlled_robot = self.robots[0]

    # ---------------------------------------------------------- 角色
    def switch_control_robot(self):
        if not self.robots:
            return
        if self.controlled_robot is None:
            self.controlled_robot = self.robots[0]
        else:
            idx = (self.robots.index(self.controlled_robot) + 1) % len(self.robots)
            self.controlled_robot = self.robots[idx]

    # ---------------------------------------------------------- 复位
    def reset(self):
        """软重置：清状态，不复位选择（controlled 指向保持不变）。"""
        for robot in self.robots:
            robot.health.reset()
            robot.gimbal.set_auto_aiming(False)
            robot.gimbal.set_auto_fire(False)
