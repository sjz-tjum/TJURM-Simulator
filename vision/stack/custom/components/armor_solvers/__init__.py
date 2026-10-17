# custom/components/armor_solvers/__init__.py
"""单板 PnP 求解器：像素四角点 -> C' 系下的板位姿。

三个实现，各自独立，没有抽象基类（三者只共用 estimate 的关键字签名，
结构相同只为了让 config 里的 sim.pnp_method 能按名字切换）：

    ippe.py    非增强 PnP：全 6 自由度      docs/design/ippe_derivation.md
    qcqp.py    方向约束的闭式解            docs/design/qcqp_derivation.md
    diff.py    差分代价的网格解            docs/design/diff_derivation.md

公共契约（三者一致）::

    estimate(*, camera_k, armor_size, image_points_2d,
             rot_camera_to_abs_camera, mount_pitch=0.0) -> (pos, rpy) | (None, None)

    image_points_2d        (4,2) 原始像素（RDF 轴序，主点在图像中心，顺序 LT/RT/RB/LB）
    rot_camera_to_abs_camera  R^{C'}_{C1}：相机系 -> 绝对方向系（= 相机在 world 下的姿态）
    mount_pitch            theta_0：板的安装俯仰（rad），ippe 不用、qcqp/diff 进模型
    返回                    pos 在 C' 系（光心为原点），rpy = [0, theta_hat_0, psi]
"""
from __future__ import annotations

_SOLVERS = ("ippe", "qcqp", "diff")


def plate_size(armor_size):
    """板几何的唯一来源：config（与 core/entities/rigids/armor.py 同源）。

    large = hero 车的大板，其余 = 小板。以前这套尺寸在求解器里另抄了一份，
    两处一旦漂开就会静默解出错深度。
    """
    from config import cfg
    veh = cfg.robot.hero if str(armor_size) == "large" else cfg.robot.infantry
    return float(veh.light_bar_interval), float(veh.light_bar_length)


def get_solver(name):
    """按 config 的 sim.pnp_method 取实现；延迟 import，避免包内循环。"""
    key = str(name)
    if key not in _SOLVERS:
        raise KeyError(f"未注册的单板求解器 {key!r}；可用：{list(_SOLVERS)}")
    if key == "ippe":
        from custom.components.armor_solvers.ippe import IppeSolver
        return IppeSolver()
    if key == "qcqp":
        from custom.components.armor_solvers.qcqp import QcqpSolver
        return QcqpSolver()
    from custom.components.armor_solvers.diff import DiffSolver
    return DiffSolver()
