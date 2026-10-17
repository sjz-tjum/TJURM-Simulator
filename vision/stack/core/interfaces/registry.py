# core/interfaces/registry.py
"""扩展点注册表：custom 的每一类扩展点共用这一套。

    class Tracker(Registry):
        kind = "跟踪器"

    @register(Tracker, "multi_target", aliases=["default"])
    class MyTracker(Tracker): ...

子类化时各自新建一份表，所以多个扩展点之间不会互相污染。
"""
from __future__ import annotations

from typing import Dict, List, Tuple

__all__ = ["Registry", "register"]


def _brief(klass) -> str:
    """一行的描述：description 优先，否则取 docstring 首行。"""
    text = (klass.description or klass.__doc__ or "").strip()
    return text.splitlines()[0] if text else ""


class Registry:
    kind: str = "实现"
    name: str = "unnamed"
    description: str = ""
    aliases: Tuple[str, ...] = ()

    _registry: Dict[str, type] = {}
    _aliases: Dict[str, str] = {}

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        cls._registry = {}
        cls._aliases = {}

    @classmethod
    def available(cls) -> List[str]:
        return sorted(set(cls._registry) | set(cls._aliases))

    @classmethod
    def canonical(cls) -> List[str]:
        return sorted(cls._registry)

    @classmethod
    def items(cls) -> List[Tuple[str, str]]:
        """[(名字, 一行描述), ...] —— 界面用；describe 也走它。"""
        return [(n, _brief(cls._registry[n])) for n in cls.canonical()]

    @classmethod
    def build(cls, name: str, **kwargs):
        key = cls._aliases.get(name, name)
        if key not in cls._registry:
            raise KeyError(f"未注册的{cls.kind} {name!r}；可用：{cls.available()}")
        return cls._registry[key](**kwargs)

    @classmethod
    def describe(cls) -> str:
        if not cls._registry:
            return f"(没有已注册的{cls.kind})"
        width = max(len(n) for n in cls._registry) + 2
        lines = [f"  {n:<{width}} {desc}" for n, desc in cls.items()]
        out = f"已注册{cls.kind}：\n" + "\n".join(lines)
        if cls._aliases:
            out += "\n别名：\n" + "\n".join(
                f"  {a} -> {t}" for a, t in sorted(cls._aliases.items()))
        return out


def register(kind_cls, name, aliases=None):
    """给某一类扩展点造注册装饰器。

    重名（主名 / 别名互相碰撞）在注册那一刻报错——重名的现象是"改了代码没反应"，
    是最难查的一类故障，早报早定位。
    """
    def _do(klass):
        if not issubclass(klass, kind_cls):
            raise TypeError(f"{klass.__name__} 必须继承 {kind_cls.__name__}")

        alias_set = tuple(aliases or ())
        challenger = f"{klass.__module__}.{klass.__qualname__}"

        # 主名：不能撞已注册的主名，也不能撞已注册的别名
        occupant = kind_cls._registry.get(name)
        if occupant is not None:
            raise ValueError(
                f"{kind_cls.kind} 主名 {name!r} 已被 "
                f"{occupant.__module__}.{occupant.__qualname__} 注册；"
                f"{challenger} 想占用同一个名字")
        alias_owner = kind_cls._aliases.get(name)
        if alias_owner is not None:
            raise ValueError(
                f"{kind_cls.kind} 主名 {name!r} 与已注册的别名冲突"
                f"（{name!r} 是 {alias_owner!r} 的别名）；"
                f"{challenger} 想占用同一个名字")

        # 别名：不能撞已注册的主名，也不能撞已注册的别名
        for a in alias_set:
            occupant = kind_cls._registry.get(a)
            if occupant is not None:
                raise ValueError(
                    f"{kind_cls.kind} 别名 {a!r} 与已注册的主名冲突"
                    f"（{a!r} 是 "
                    f"{occupant.__module__}.{occupant.__qualname__} 的主名）；"
                    f"{challenger} 想用它作别名")
            owner = kind_cls._aliases.get(a)
            if owner is not None:
                raise ValueError(
                    f"{kind_cls.kind} 别名 {a!r} 已被 {owner!r} 占用；"
                    f"{challenger} 想占用同一个别名")

        klass.name = name
        klass.aliases = alias_set
        kind_cls._registry[name] = klass
        for a in alias_set:
            kind_cls._aliases[a] = name
        return klass
    return _do