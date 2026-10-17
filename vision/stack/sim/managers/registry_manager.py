# sim/managers/registry_manager.py
"""注册表管理器：按"运动更新方式"给在场的物理对象分桶。

给 MotionManager 提供四份遍历列表：
    chassis  —— 车的底盘，物理步进 + update_children
    gimbals  —— 车的云台，apply_control 或物理步进 + update_children
    outposts —— 前哨站，kinematic update
    runes    —— 能量机关，kinematic update

分类口径是「运动更新方式」，不是「实体种类」。
CHASSIS / GIMBAL 是车的部件，OUTPOST / RUNE 是独立设备。

职责边界：只做"存 + 给"——不建实体、不定运动、不做业务判断、不持状态。
"""
from core.entities.property.entity_types import RegistryType


class RegistryManager:
    def __init__(self):
        self.chassis = []
        self.gimbals = []
        self.outposts = []
        self.runes = []

    def add(self, entity, kind):
        bucket = self._bucket(kind)
        if entity not in bucket:
            bucket.append(entity)

    def remove(self, entity):
        for bucket in self._buckets():
            if entity in bucket:
                bucket.remove(entity)
                return True
        return False

    def _buckets(self):
        return (self.chassis, self.gimbals, self.outposts, self.runes)

    def _bucket(self, kind):
        if kind is RegistryType.CHASSIS:
            return self.chassis
        if kind is RegistryType.GIMBAL:
            return self.gimbals
        if kind is RegistryType.OUTPOST:
            return self.outposts
        if kind is RegistryType.RUNE:
            return self.runes
        raise KeyError(f"未知实体类别 {kind!r}")

    def __repr__(self):
        return (f"RegistryManager(chassis={len(self.chassis)}, "
                f"gimbals={len(self.gimbals)}, "
                f"outposts={len(self.outposts)}, "
                f"runes={len(self.runes)})")
