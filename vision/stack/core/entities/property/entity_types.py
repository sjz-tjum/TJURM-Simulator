# core/entities/property/entity_types.py
"""实体类型定义：三个正交的标签维度。

EntityType   —— 实体的种类（Hero/Sentry/.../Rune），从 YAML 字符串解析。
TargetType   —— 跟踪预测的分类（ROBOT/OUTPOST/RUNE），用于 sys_bus 主题后缀。
RegistryType —— 运动更新队列的分类（CHASSIS/GIMBAL/OUTPOST/RUNE），
                用于 RegistryManager 分桶。注意 CHASSIS/GIMBAL 是车的部件，
                不是实体本身；OUTPOST/RUNE 是实体本身。
"""
from enum import Enum, IntEnum


class EntityType(IntEnum):
    Hero = 1
    Sentry = 2
    Infantry = 3
    Engineer = 4
    Outpost = 5
    Base = 6
    Rune = 7

    @classmethod
    def get_name(cls, entity_type) -> str:
        """把实体标识规范成配置里的键（小写的段名/车型名）。"""
        if isinstance(entity_type, cls):
            return entity_type.name.lower()
        if isinstance(entity_type, str):
            s = entity_type.strip()
            if not s:
                raise ValueError("entity_type 不能为空")
            for m in cls:
                if m.name.lower() == s.lower():
                    return m.name.lower()
            try:
                return cls(int(s)).name.lower()
            except (ValueError, KeyError):
                pass
        try:
            return cls(int(entity_type)).name.lower()
        except (ValueError, KeyError, TypeError):
            raise ValueError(f"未知 entity_type={entity_type!r}")

    @classmethod
    def coerce(cls, entity_type) -> "EntityType":
        """把任意合法写法转成枚举成员。"""
        name = cls.get_name(entity_type)
        for m in cls:
            if m.name.lower() == name:
                return m
        raise ValueError(f"未知 entity_type={entity_type!r}")


class TargetType(Enum):
    """打谁：分通道的粒度。"""
    ROBOT = "robot"
    OUTPOST = "outpost"
    RUNE = "rune"


class RegistryType(Enum):
    """运动更新队列的分类。"""
    CHASSIS = "chassis"
    GIMBAL = "gimbal"
    OUTPOST = "outpost"
    RUNE = "rune"
