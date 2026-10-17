# custom/__init__.py
"""客户算法包。

discover() + boot() = 发现并注册 custom/ 下的全部实现、拼出整条链（Pipeline）。
由入口 entrance/ 调用，拼好的 Pipeline 交给 entrance 的 keeper 用 tick() 驱动。

custom 不认识 sim，sim 也不认识 custom —— 中间只隔 core 的接口和总线。
"""
from __future__ import annotations

import importlib
import pkgutil

__all__ = ["discover", "ensure_registered", "boot", "shutdown"]

_state = {"pipeline": None}


def discover() -> None:
    """扫 custom/ 整棵树，import 每个模块，触发 @register_* 装饰器。"""
    prefix = __name__ + "."
    for _finder, name, _is_pkg in pkgutil.walk_packages(__path__, prefix=prefix):
        if any(part.startswith("_") for part in name.split(".")):
            continue
        importlib.import_module(name)


def ensure_registered() -> None:
    """发现 custom 实现。任何要用注册表的入口都先调它。"""
    discover()


def _pick(kind_cls, name):
    if name is None:
        names = kind_cls.canonical()
        if not names:
            raise RuntimeError(f"没有已注册的{kind_cls.kind}")
        name = names[0]
    if name not in kind_cls.available():
        raise RuntimeError(f"未注册的{kind_cls.kind} {name!r}；"
                           f"可用：{kind_cls.available()}")
    return name


def boot(*, comm,
         tracker_name="multi_target",
         pose_solver_name="tjurm"):
    """发现实现、拼出 Pipeline，返回给入口交给 keeper 驱动。"""
    ensure_registered()

    from core.interfaces.tracker import Tracker
    from core.interfaces.pose_solver import PoseSolver

    tracker = Tracker.build(_pick(Tracker, tracker_name))
    pose_solver = PoseSolver.build(_pick(PoseSolver, pose_solver_name))

    from custom.pipeline import Pipeline
    pipeline = Pipeline(tracker=tracker, pose_solver=pose_solver, comm=comm)
    _state["pipeline"] = pipeline
    return pipeline


def shutdown() -> None:
    pipeline = _state.get("pipeline")
    if pipeline is not None:
        pipeline.reset()
    _state["pipeline"] = None