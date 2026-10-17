# core/entities/property/health.py
"""血量：受击扣血，归零即阵亡。"""
from typing import Optional


class Health:
    def __init__(self, max_hp: float, hp: Optional[float] = None):
        self.max_hp = float(max_hp)
        self.hp = float(max_hp if hp is None else hp)
        self.alive = self.hp > 0.0
        self.last_hit_time = None

    def take_damage(self, amount: float, t: Optional[float] = None) -> float:
        """扣血并返回实际伤害（已阵亡则返回 0）。"""
        if not self.alive:
            return 0.0
        dealt = min(self.hp, max(0.0, float(amount)))
        self.hp -= dealt
        self.last_hit_time = t
        if self.hp <= 0.0:
            self.hp = 0.0
            self.alive = False
        return dealt

    def reset(self) -> None:
        self.hp = self.max_hp
        self.alive = self.hp > 0.0
        self.last_hit_time = None

    @property
    def ratio(self) -> float:
        return 0.0 if self.max_hp <= 0 else self.hp / self.max_hp