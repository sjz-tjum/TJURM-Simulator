# yaw_pnp.py —— 移植用户那版「YawPnP」：把 yaw 当唯一未知量做一维搜索
#
# 为什么要它（2026-10-12，用户实测"静止靶输出 yaw 乱甩" ✓）：
#   常规做法是每块板做 6-DoF PnP。共面 4 点在**正对相机**时有两个近乎等价的镜像解 ✗，
#   求解器逐帧在两者间跳 ⇒ 板法线翻 180° ⇒ 整车 yaw 跳几十度 ⇒ 输出乱甩 ✓✓。
#   用户给的那版老代码（能稳定瞄准 ✓）把 yaw 当**一维**未知量，用三分搜索最小化
#   「线段长度差 + 线段夹角」代价 ⇒ 单峰 ⇒ 不存在翻转 ✓✓。
#
# 本文件忠实移植其核心：
#   1) cv2.solvePnP(IPPE) 只用来拿**位置**（tvec ✓，位置不受 yaw 二义性影响 ✓）
#   2) 在 yaw ∈ [-90°, +90°] 上三分搜索，代价 = Σ(线段长度差归一化) + ANGLE_COST_RATIO*Σ(线段夹角)
#   3) 近似其 getYawByMix：|yaw| 在 0.6±0.05 rad 附近时把"像素代价解"和"角度代价解"混合 ✓
#
# 坐标约定（与 PlugYourRM 一致 ✓）：
#   世界/云台系：x 前、y 左、z 上（右手 ✓，yaw 逆时针为正 ✓）
#   板模型：原点在板中心，x = 板面法线（朝观测者），y = 板宽方向，z = 板高方向 ✓
#   相机：K 主点在图像中心 ✓、像素 x 向右、y 向下（RDF ✓）

from __future__ import annotations

import math

import cv2
import numpy as np

ANGLE_COST_RATIO = 4.0          # 与用户源码一致 ✓
MIX_MID, MIX_HALF = 0.6, 0.05   # getYawByMix 的参数 ✓


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _plate_model(w, h):
    """板的四个模型点（LT/RT/RB/LB ✓ 与 Detection 的顺序一致 ✓）。

    约定：板面法线 = +x（朝相机 ✓）、y 向板宽、z 向板高 ✓。
    法线为 +x ⇒ 板平面是 y-z 平面 ✓。
    """
    hw, hh = w / 2.0, h / 2.0
    # LT/RT/RB/LB：从相机看，左是 +y（世界 y 向左 ✓）
    return np.array([[0.0, +hw, +hh],
                     [0.0, -hw, +hh],
                     [0.0, -hw, -hh],
                     [0.0, +hw, -hh]], float)


def solve_yaw(
    K_px: np.ndarray,
    pts_px: np.ndarray,
    plate_w: float,
    plate_h: float,
    cam_rpy: np.ndarray,
    cam_pos_world: np.ndarray,
    armor_tilt_deg: float = 15.0,
    yaw_center: float = 0.0,
    span_deg: float = 90.0,
    eps_deg: float = 0.6,
):
    """返回 (yaw_world_rad, pos_world, info)。

    K_px   : 3x3，**主点在图像中心**（他们的约定 ✓）
    pts_px : 4x2，以图像中心为原点的像素（LT/RT/RB/LB ✓，x 右 y 下 ✓）
    cam_rpy: 相机在**世界系**的 rpy（rad，x-roll/y-pitch/z-yaw ✓ 抬头为正 ✓）
    yaw_center: 搜索中心（一般传相机的世界 yaw ✓）
    """
    pts = np.asarray(pts_px, float).reshape(4, 2)
    model = _plate_model(plate_w, plate_h)

    # ---- 1) 用 IPPE 拿位置（位置与 yaw 二义性无关 ✓）----
    ok, rvec, tvec = cv2.solvePnP(model, pts, np.asarray(K_px, float), None,
                                  flags=cv2.SOLVEPNP_IPPE)
    bad = (not ok) or (tvec is None) or (not np.all(np.isfinite(tvec)))
    if not bad:
        # IPPE 解出的深度必须在相机前方 ✓（否则是镜像解 ✗）
        t_rdf0 = np.asarray(tvec, float).reshape(3)
        bad = t_rdf0[2] <= 1e-3
    if bad:
        # ★ 兜底（verify_yawpnp.py 实测：部分样本 IPPE 直接给 NaN ✗ ⇒ 必须自己给距离）
        # 用"板的视宽"估深度：z ≈ f * W / w_px（正面近似 ✓），只在 PnP 失败时用 ✓
        w_px = float(np.linalg.norm(pts[1] - pts[0]))
        if w_px < 1e-6:
            return None, None, {"stage": "pnp failed and no width"}
        z_est = float(np.asarray(K_px, float)[0, 0]) * plate_w / w_px
        t_flu = np.array([z_est, 0.0, 0.0], float)
        # 用四角点中心给一个横向/纵向的一阶修正 ✓
        c = pts.mean(axis=0)
        t_flu = np.array([z_est, -c[0] * z_est / float(np.asarray(K_px, float)[0, 0]),
                          -c[1] * z_est / float(np.asarray(K_px, float)[1, 1])], float)
    else:
        t_rdf = np.asarray(tvec, float).reshape(3)
        t_flu = np.array([t_rdf[2], -t_rdf[0], -t_rdf[1]], float)

    # 相机系 FLU：把 PnP 的 RDF 结果换回 FLU（他们的 vec3d_RDF2FLU ✓）
    #   RDF = (右, 下, 前) ⇒ FLU = (前, 左, 上) = (z_rdf, -x_rdf, -y_rdf)
    t_rdf = tvec.reshape(3)
    t_flu = np.array([t_rdf[2], -t_rdf[0], -t_rdf[1]], float)

    R_wc = _Rz(cam_rpy[2]) @ _Ry(cam_rpy[1]) @ _Rz(cam_rpy[0])   # 相机 -> 世界（ZYX ✓）
    pos_world = R_wc @ t_flu + np.asarray(cam_pos_world, float)

    # ---- 2) 一维三分搜索定 yaw（单峰 ⇒ 不会翻转 ✓）----
    tilt = math.radians(armor_tilt_deg)

    def project(yaw: float) -> np.ndarray:
        # 板在世界系的姿态：法线朝"从相机看过去"的方向 ✓ ⇒ 用 yaw 和倾角构造 ✓
        R_plate = _Rz(yaw) @ _Ry(+tilt)   # 对齐他们 getMapping：col0 = (cos·cos, sin·cos, sin(pitch)) ✓
        Pw = (R_plate @ model.T).T + pos_world          # 4x3 世界点 ✓
        # 世界(FLU) -> 相机：R_wc^T，再 FLU -> RDF ✓
        Pc = (R_wc.T @ (Pw - np.asarray(cam_pos_world, float)).T).T
        Prdf = np.stack([-Pc[:, 1], -Pc[:, 2], Pc[:, 0]], axis=1)   # x=右(-y_left) ✓
        z = np.clip(Prdf[:, 2], 1e-6, None)
        uv = (np.asarray(K_px, float) @ Prdf.T).T[:, :2] / z[:, None]
        return uv

    def cost_pixel(yaw: float) -> float:
        pj = project(yaw)
        c = 0.0
        for i in range(4):
            j = (i + 1) % 4
            pl, ql = pts[j] - pts[i], pj[j] - pj[i]
            nl = max(np.linalg.norm(pl), 1e-6)
            c += (0.5 * (np.linalg.norm(pts[i] - pj[i]) + np.linalg.norm(pts[j] - pj[j]))
                  + abs(np.linalg.norm(pl) - np.linalg.norm(ql))) / nl
        return c

    def cost_angle(yaw: float) -> float:
        pj = project(yaw)
        c = 0.0
        for i in range(4):
            j = (i + 1) % 4
            pl, ql = pts[j] - pts[i], pj[j] - pj[i]
            d = np.linalg.norm(pl) * np.linalg.norm(ql)
            c += abs(math.acos(float(np.clip(np.dot(pl, ql) / max(d, 1e-9), -1.0, 1.0))))
        return c

    def tern(f, lo, hi, eps):
        while hi - lo > eps:
            m1 = lo + (hi - lo) / 3.0
            m2 = hi - (hi - lo) / 3.0
            if f(m1) < f(m2):
                hi = m2
            else:
                lo = m1
        return 0.5 * (lo + hi)

    eps = math.radians(eps_deg)
    lo, hi = yaw_center - math.radians(span_deg), yaw_center + math.radians(span_deg)
    yaw_px = tern(cost_pixel, lo, hi, eps)
    yaw_ang = tern(cost_angle, lo, hi, eps)
    # getYawByMix 的近似 ✓
    ap = yaw_px - yaw_center
    if abs(ap) <= MIX_MID - MIX_HALF:
        yaw = yaw_ang
    elif abs(ap) >= MIX_MID + MIX_HALF:
        yaw = yaw_px
    else:
        r = 0.5 + 0.5 * math.sin(math.pi * (abs(ap) - MIX_MID) / (2 * MIX_HALF))
        yaw = r * yaw_px + (1 - r) * yaw_ang
    return yaw, pos_world, {"yaw_px": yaw_px, "yaw_ang": yaw_ang, "t_flu": t_flu}
