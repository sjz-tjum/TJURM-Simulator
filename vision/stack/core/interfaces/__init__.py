# core/interfaces/__init__.py
"""跨模块接口契约：插件需要实现的基类与注册表入口。"""
from core.interfaces.registry import Registry, register  # noqa: F401
from core.interfaces.tracker import Tracker, register_tracker  # noqa: F401
from core.interfaces.pose_solver import (  # noqa: F401
    PoseSolver, register_pose_solver,
)