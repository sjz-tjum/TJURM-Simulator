# gt_calib.py —— 真值标定：用**已知几何**喂 PlugYourRM 的管线，看它解出的角度对不对
#
# 为什么要它（2026-10-11）：现场表现是"pitch 一进自瞄就压到底、yaw 不跟"，而离线检测已经 100% ✓，
# 所以嫌疑只剩两处：① 我喂的 SelfState（世界系位姿 + 内参）约定不对；② 他们的 config 几何与仿真不一致。
# 这个脚本把几何**完全已知**：相机在原点、朝 +x、水平；靶板放在指定的世界位置并正对相机；
# 四角点用真值投影算出来。于是"他们解出的 yaw/pitch"必须等于几何真值 —— 差多少、差在哪一步，一目了然。
#
# 用法：python -X utf8 -u gt_calib.py [--x 3.0] [--y -0.4] [--z 0.2] [--plate-w 0.133] [--plate-h 0.05]
#   x = 前方距离(m)；y = **左为正**（负=在右边）；z = 高于相机(m)
#   预期：yaw ≈ atan2(y, x)（他们 yaw 逆时针为正、y 轴朝左）；pitch ≈ atan2(z, x)（抬头为正）

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

PLUGYOU = r"D:\PlugYourRM"
if PLUGYOU not in sys.path:
    sys.path.insert(0, PLUGYOU)

F_PX = 1250.0            # 我们的自瞄相机：1280x1024 + 6mm
CX, CY = 0.0, 0.0        # 他们的约定：主点是图像中心 = (0,0)
K = np.array([[F_PX, 0, 0], [0, F_PX, 0], [0, 0, 1]], float)


def project_world_to_centered_px(P):
    """世界系(FLU: x前/y左/z上) -> 他们约定的"以图像中心为原点的像素"(RDF: 右/下/前)。

    相机在原点、朝 +x、水平（rpy=0）。相机系 FLU 与世界系重合 ⇒
    RDF = (右, 下, 前) = (-y, -z, +x)，投影 u = f·X/Z, v = f·Y/Z。
    """
    X = -P[1]
    Y = -P[2]
    Z = P[0]
    return np.array([F_PX * X / Z, F_PX * Y / Z], float)


def plate_corners(center, w, h):
    """正对相机（法线朝 -x）的板四角，顺序 LT/RT/RB/LB（在世界系里给）。"""
    cx, cy, cz = center
    return [np.array([cx, cy + w / 2, cz + h / 2]),   # LT（板面从相机看：左=世界 +y ✓）
            np.array([cx, cy - w / 2, cz + h / 2]),   # RT
            np.array([cx, cy - w / 2, cz - h / 2]),   # RB
            np.array([cx, cy + w / 2, cz - h / 2])]   # LB


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=3.0)
    ap.add_argument("--y", type=float, default=-0.4, help="左为正；负=在右边")
    ap.add_argument("--z", type=float, default=0.2, help="高于相机")
    ap.add_argument("--plate-w", type=float, default=0.133)
    ap.add_argument("--plate-h", type=float, default=0.05)
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--armor-size", default="small")
    args = ap.parse_args()

    # ---- 装总线 + boot 他们的 custom（和 plugyou_peer 一样）----
    from core.bus import EventBus, install_sys_bus, install_public_bus, public_bus
    install_sys_bus(EventBus())
    install_public_bus(EventBus())
    from custom.comm import CustomComm
    from core.bus.messages import PublicMessages
    comm = CustomComm()
    import custom
    pipeline = custom.boot(comm=comm, tracker_name="multi_target", pose_solver_name="tjurm")

    solved = {"n": 0, "last": None}
    public_bus().subscribe(PublicMessages.CONTROL, lambda s: (solved.update(n=solved["n"] + 1, last=s)))

    from core.bus.dataflow import Detection, DetectionFrame, SelfState, RigidState, CameraState
    from core.entities.property.entity_types import EntityType

    # ---- 真值 ----
    center = (args.x, args.y, args.z)
    corners = plate_corners(center, args.plate_w, args.plate_h)
    px = np.array([project_world_to_centered_px(c) for c in corners], float)
    yaw_truth = math.degrees(math.atan2(args.y, args.x))
    pitch_truth = math.degrees(math.atan2(args.z, args.x))
    print("真值: 板中心=(%.2f, %.2f, %.2f) m  真值 yaw=%+.2f deg  pitch=%+.2f deg"
          % (args.x, args.y, args.z, yaw_truth, pitch_truth))
    print("真值投影四角(以图像中心为原点的像素):")
    for nm, q in zip("LT RT RB LB".split(), px):
        print("   %s = (%+8.2f, %+8.2f)" % (nm, q[0], q[1]))

    zero = np.zeros(3)
    for i in range(args.frames):
        t = i / 30.0
        frame = DetectionFrame(
            detections=[Detection(entity_type=EntityType.Infantry,
                                  image_points_2d=px.copy(), armor_size=args.armor_size)],
            timestamp=t)
        ss = SelfState(
            chassis=RigidState(pos=zero.copy(), rpy=zero.copy(), vel=zero.copy(), omg=zero.copy()),
            gimbal=RigidState(pos=zero.copy(), rpy=zero.copy(), vel=zero.copy(), omg=zero.copy()),
            muzzle=RigidState(pos=zero.copy(), rpy=zero.copy(), vel=zero.copy(), omg=zero.copy()),
            camera=CameraState(pos=zero.copy(), rpy=zero.copy(), K=K.copy()))
        public_bus().publish(PublicMessages.DETECTION, frame)
        public_bus().publish(PublicMessages.SELF_STATE, ss)
        pipeline.tick()

    s = solved["last"]
    print("\n他们的输出: %d 个 ShotSolution" % solved["n"])
    if s is None:
        print("  ✗ 没有任何解 —— 说明他们的 pose_solver/tracker 在这个输入下直接失败")
        return
    print("  yaw=%+.2f deg  pitch=%+.2f deg  fire=%s" % (math.degrees(s.yaw), math.degrees(s.pitch), s.fire))
    print("  误差:      Δyaw=%+.2f deg   Δpitch=%+.2f deg"
          % (math.degrees(s.yaw) - yaw_truth, math.degrees(s.pitch) - pitch_truth))


if __name__ == "__main__":
    main()
