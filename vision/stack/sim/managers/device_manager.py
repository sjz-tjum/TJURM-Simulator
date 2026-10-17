# sim/managers/device_manager.py
"""设施管理器：前哨站、能量机关。单实例，幂等增删。

RM 里只有一个前哨站、一个能量机关，所以是两个单实例而不是列表。
与 RobotManager 的列表形态不对称，这是设计选择，不是遗漏。
"""
from __future__ import annotations

import numpy as np

from core.entities.property.entity_types import EntityType, RegistryType
from core.entities.rigids.outpost import Outpost
from core.entities.rigids.rune import Rune


class DeviceManager:
    def __init__(self, registry, rng=None):
        self.registry = registry
        self.outpost = None
        self.rune = None
        self._rng = rng if rng is not None else np.random.default_rng()

    # ---------------------------------------------------------- 增删
    def add_device(self, entity_type, world_pos=None):
        """幂等：已存在就直接返回，键盘按多了不崩。

        入参先过 EntityType.coerce：与 RobotManager.add_robot 的口径一致，
        写成 "Outpost" / EntityType.Outpost / 5 都能用。少了这一步，脚本里
        传字符串就会在这里炸，而隔壁 add_robot 却好好的——同一份代码里两种
        口径是纯粹的坑，不是设计。
        """
        entity_type = EntityType.coerce(entity_type)
        if entity_type is EntityType.Outpost:
            if self.outpost is not None:
                return
            self.outpost = Outpost(world_pos=world_pos)
            self.registry.add(self.outpost, RegistryType.OUTPOST)
        elif entity_type is EntityType.Rune:
            if self.rune is not None:
                return
            self.rune = Rune(world_pos=world_pos, rng=self._rng)
            self.registry.add(self.rune, RegistryType.RUNE)
        else:
            raise ValueError(f"DeviceManager 不接受实体类型 {entity_type!r}")

    def remove_device(self, entity_type):
        """幂等：不存在就直接返回。registry 的解除由本类调 registry.remove 完成；
        本类同时清掉自身引用。"""
        entity_type = EntityType.coerce(entity_type)
        if entity_type is EntityType.Outpost:
            if self.outpost is None:
                return
            self.registry.remove(self.outpost)
            self.outpost = None
        elif entity_type is EntityType.Rune:
            if self.rune is None:
                return
            self.registry.remove(self.rune)
            self.rune = None
        else:
            raise ValueError(f"DeviceManager 不接受实体类型 {entity_type!r}")

    # ---------------------------------------------------------- 能量机关操作
    def switch_rune_mode(self):
        if self.rune is None:
            return
        self.rune.set_mode(not self.rune.is_big_rune)

    def trigger_rune(self):
        """KP6：一次按开、再按关。"""
        if self.rune is None:
            return
        if self.rune.armed:
            self.rune.cancel()
        else:
            self.rune.trigger(self.rune.world_t)

    # ---------------------------------------------------------- 随机源
    def set_rng(self, rng):
        self._rng = rng
        if self.rune is not None:
            self.rune.set_rng(rng)

    # ---------------------------------------------------------- 重置
    def reset(self):
        if self.outpost is not None:
            self.outpost.health.reset()
        if self.rune is not None:
            self.rune.reset()
