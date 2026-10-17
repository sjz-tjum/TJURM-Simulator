# core/algorithms/control/__init__.py
"""控制器族：PID / MPC。"""
from core.algorithms.control.pid import (  # noqa: F401
    PIDParams, PID, CascadedPID,
)
from core.algorithms.control.mpc import (  # noqa: F401
    MPCConfig, ProjectedGradientQP, LinearGimbalMPC,
)