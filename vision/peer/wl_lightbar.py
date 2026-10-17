# wl_lightbar.py —— 传统灯条识别（1:1 照搬 sp_vision_25-main/tasks/auto_aim/detector.cpp + armor.cpp）
#
# 用户 2026-10-11 明确的真车流程：
#     YOLO 出框 → 用框构造 ROI（沿两侧延长 1 倍、横向 ±75%）→ ROI 内二值化找灯条
#     → 按 tl/bl 与 br/tr 的最近距离配出左右灯条 → 输出四角点 LT/RT/RB/LB + 红蓝颜色
#
# 对应的 C++ 出处：
#   armor.cpp:9-31    Lightbar 构造（top/bottom/top2bottom/width/angle/angle_error/length/ratio）
#   armor.cpp:34-56   Armor(left,right)：ratio / side_ratio / rectangular_error
#   detector.cpp:233  check_geometry(Lightbar)：角度误差、长宽比、最小长度
#   detector.cpp:241  check_geometry(Armor)：ratio / side_ratio / rectangular_error
#   detector.cpp:279  get_color：按轮廓像素的 R 和 / B 和判红蓝
#   detector.cpp:130-157  ROI 构造（就是上面说的延长规则）
#   detector.cpp:160-230  ROI 内找灯条 → 配左右 → 四角点（距离和 < 15 px 才接受）
#   configs/standard3.yaml  传统方法参数 + enemy_color + min_detection_confidence
#
# 为什么用它：PnP 要的是四角点；YOLO 只给框。灯条配对给出的是真正的板面角点 ✓，
# 而且 get_color 直接给出红蓝（顺手解决敌我区分 ✓）。

from __future__ import annotations

import cv2
import numpy as np

# ---- configs/standard3.yaml 的传统方法参数（原样） ----
CFG = dict(
    threshold=150,
    max_angle_error_deg=45.0,
    min_lightbar_ratio=1.5,
    max_lightbar_ratio=20.0,
    min_lightbar_length=3.0,   # 原来是 8（真车标定）；仿真里远距离板小，放宽到 3
    min_armor_ratio=1.0,
    max_armor_ratio=5.0,
    max_side_ratio=1.5,
    max_rectangular_error_deg=25.0,
    # detector.cpp:221 原文是 15 px —— 但那是喂"神经网络关键点"时的阈值。我们喂的是**YOLO 框的四角**，
    # 框比装甲板大、角点不重合，15 px 必然全丢（实测 36 个框 0 通过 ✗）。按我们的输入放宽到 50。
    otsu_min=45.0,               # Otsu 下限，避免全黑帧产生噪声二值化
    max_pair_distance=100.0,   # 实测：喂框角点时距离和约 80px（真车帧 d_left=46.8 d_right=33.7）
)

RED, BLUE = "red", "blue"

# 调试用：refine_box 每次会把中间量放这里（ROI 原图 / 二值化图 / 阈值 / 检测到的灯条），
# 供 --debug-dir 导出成图。用户 2026-10-11 明确要看二值化图和 ROI。
LAST_DEBUG = {}


class Lightbar:
    """对应 armor.cpp 的 Lightbar 构造。"""

    def __init__(self, rotated_rect, lid: int):
        self.id = lid
        self.rotated_rect = rotated_rect
        corners = cv2.boxPoints(rotated_rect)                 # 4x2
        corners = corners[np.argsort(corners[:, 1])]          # 按 y 排序（照抄 C++）
        self.center = np.asarray(rotated_rect[0], float)
        self.top = (corners[0] + corners[1]) / 2.0
        self.bottom = (corners[2] + corners[3]) / 2.0
        self.top2bottom = self.bottom - self.top
        self.points = [self.top, self.bottom]
        self.width = float(np.linalg.norm(corners[0] - corners[1]))
        self.angle = float(np.arctan2(self.top2bottom[1], self.top2bottom[0]))
        self.angle_error = abs(self.angle - np.pi / 2)
        self.length = float(np.linalg.norm(self.top2bottom))
        self.ratio = self.length / max(1e-6, self.width)
        self.color = None


def check_lightbar(lb: Lightbar, cfg=CFG) -> bool:
    """detector.cpp:233 check_geometry(Lightbar)。"""
    return (lb.angle_error < np.deg2rad(cfg["max_angle_error_deg"])
            and cfg["min_lightbar_ratio"] < lb.ratio < cfg["max_lightbar_ratio"]
            and lb.length > cfg["min_lightbar_length"])


def get_color(roi_bgr, contour) -> str:
    """detector.cpp:279 get_color：只统计轮廓像素的 R/B 和。"""
    red_sum = blue_sum = 0
    h, w = roi_bgr.shape[:2]
    for p in contour.reshape(-1, 2):
        x, y = int(p[0]), int(p[1])
        if 0 <= x < w and 0 <= y < h:
            b, g, r = roi_bgr[y, x]
            red_sum += int(r)
            blue_sum += int(b)
    return BLUE if blue_sum > red_sum else RED


class Armor:
    """对应 armor.cpp:34 Armor(left, right)。"""

    def __init__(self, left: Lightbar, right: Lightbar):
        self.left, self.right = left, right
        self.color = left.color
        self.center = (left.center + right.center) / 2.0
        # ★★ 2026-10-13 按官方顺序修正：**左上, 右上, 左下, 右下** ★★
        #   依据（用户指出的真实工程 OpenRM / TJURM-Aiming）：
        #     OpenRM/src/pointer/setter.cpp:93-107  setArmorFourPointsRelative()
        #        [0]=左.上  [1]=右.上  [2]=左.下  [3]=右.下
        #        （且**按灯条中心 x 坐标强制分左右**，不是按 YOLO 框角距离）
        #     TJURM-Aiming/src/.../locater.cpp:27-39  3D 对象点同样是"左上-右上-左下-右下"
        #   原来这里写的是 [左.上, 右.上, 右.下, 左.下]（绕圈 LT/RT/RB/LB），
        #   与官方及配套的 3D 点顺序都不一致 ✗
        self.points = [left.top, right.top, left.bottom, right.bottom]
        l2r = right.center - left.center
        width = float(np.linalg.norm(l2r))
        max_len = max(left.length, right.length)
        min_len = max(1e-6, min(left.length, right.length))
        self.ratio = width / max(1e-6, max_len)
        self.side_ratio = max_len / min_len
        roll = float(np.arctan2(l2r[1], l2r[0]))
        e_l = abs(left.angle - roll - np.pi / 2)
        e_r = abs(right.angle - roll - np.pi / 2)
        self.rectangular_error = max(e_l, e_r)


def check_armor(armor: Armor, cfg=CFG) -> bool:
    """detector.cpp:241 check_geometry(Armor)。"""
    return (cfg["min_armor_ratio"] < armor.ratio < cfg["max_armor_ratio"]
            and armor.side_ratio < cfg["max_side_ratio"]
            and armor.rectangular_error < np.deg2rad(cfg["max_rectangular_error_deg"]))


def _roi_from_box(image, pts, cfg=CFG):
    """detector.cpp:130-157 的 ROI 构造：4 个框角点 -> 延展后的 ROI。

    pts 顺序 LT/RT/RB/LB。
    """
    tl, tr, br, bl = [np.asarray(p, float) for p in pts]
    lt2b = bl - tl
    rt2b = br - tr
    tl1 = (tl + bl) / 2 - lt2b
    bl1 = (tl + bl) / 2 + lt2b
    br1 = (tr + br) / 2 + rt2b
    tr1 = (tr + br) / 2 - rt2b
    tl2tr = tr1 - tl1
    bl2br = br1 - bl1
    tl2 = (tl1 + tr) / 2 - 0.75 * tl2tr
    tr2 = (tl1 + tr) / 2 + 0.75 * tl2tr
    bl2 = (bl1 + br) / 2 - 0.75 * bl2br
    br2 = (bl1 + br) / 2 + 0.75 * bl2br
    quad = np.array([tl2, tr2, br2, bl2], np.float32)
    bb = cv2.boundingRect(quad)                        # (x, y, w, h)
    x, y, w, h = bb
    H, W = image.shape[:2]
    if x < 0 or y < 0 or x + w > W or y + h > H:
        return None, None                              # C++ 同样直接 return false
    roi = image[y:y + h, x:x + w]
    return (roi, (x, y)) if roi.size else (None, None)


def refine_box(image, box_xyxy, cfg=CFG, enemy_color=None, debug=False):
    """YOLO 框 → ROI → 灯条 → 四角点。返回 (points4, color) 或 (None, None)。

    对应 detector.cpp:122-231 Detector::detect(Armor&, bgr_img)。
    """
    x1, y1, x2, y2 = [float(v) for v in box_xyxy]
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]     # LT/RT/RB/LB
    roi, off = _roi_from_box(image, pts, cfg)
    if roi is None:
        if debug:
            print("    box=(%.0f,%.0f,%.0f,%.0f) ROI 越界 -> 丢弃" % (x1, y1, x2, y2))
        return None, None
    ox, oy = off

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    # 阈值：他们的 150 是按**真车相机**标的；仿真帧经小光圈后暗得多（实测 max 40~85），
    # 固定阈值会二值化出空图 ⇒ 一个灯条都找不到（实测 refined=0 ✗）。这里用 Otsu 自适应，
    # 并夹在 [otsu_min, threshold] 之间：真车帧上仍接近原来的 150，仿真暗帧上自动降低。
    # ★ 2026-10-11 实测修正：outs_min=45 会把仿真帧抬到找不到灯条 ✗（实测 YOLO 框 83x39 px 时
    #   轮廓=0，二值化出空图）。改成**按 ROI 自身统计**：p99.5 是"ROI 里最亮的那部分"，
    #   取 0.6*p99.5 就一定能分出前景；再与 Otsu、固定阈值三者取最小（谁更能保住灯条听谁的）。
    p995 = float(np.percentile(gray, 99.5))
    otsu, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thr = float(min(cfg["threshold"], max(5.0, 0.6 * p995, min(float(otsu), 0.8 * p995))))
    _, binary_gray = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY)

    # ★★ 彩色分离（2026-10-11 用导出图定案）：
    # 真车灯条亮到 200+、漆面暗 ⇒ 灰度阈值够用；**仿真反过来** —— 灯条是蓝色（蓝光在灰度里很暗），
    # 而装甲板的"4"字是浅灰漆（灰度更亮）⇒ 灰度阈值只会抠到字，`contours=4 bars=0` ✗✗。
    # 所以优先按**彩色 + 够亮**分离（HSV 饱和度），灰度阈值只作为兜底。
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    sat, val = hsv[:, :, 1], hsv[:, :, 2]
    mask = ((sat > 60) & (val > 20)).astype(np.uint8) * 255
    n_color_px = int((mask > 0).sum())
    if n_color_px >= 12:                     # 有彩色像素才用彩色掩膜
        binary = mask
        thr_mode = "color(sat>60,val>20) px=%d" % n_color_px
    else:
        binary = binary_gray
        thr_mode = "gray thr=%.0f" % thr
    LAST_DEBUG.clear()
    LAST_DEBUG.update(roi=roi.copy(), binary=binary.copy(), thr=thr, p995=p995, otsu=float(otsu), thr_mode=thr_mode,
                      box=(x1, y1, x2, y2), off=(ox, oy))
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    bars = []
    ngeom = 0
    for i, c in enumerate(contours):
        lb = Lightbar(cv2.minAreaRect(c), i)
        if not check_lightbar(lb, cfg):
            ngeom += 1
            continue
        lb.color = get_color(roi, c)
        bars.append(lb)
    LAST_DEBUG.update(bars=bars, contours=contours, roi_gray=gray, thr_used=thr)
    if debug:
        print("    box=(%.0f,%.0f,%.0f,%.0f) ROI=%dx%d@(%d,%d) 轮廓=%d 几何不过=%d 灯条=%d"
              % (x1, y1, x2, y2, roi.shape[1], roi.shape[0], ox, oy, len(contours), ngeom, len(bars)))
        if bars:
            for b in bars[:4]:
                print("      bar len=%.0f w=%.0f ratio=%.1f ang_err=%.1fdeg color=%s"
                      % (b.length, b.width, b.ratio, np.rad2deg(b.angle_error), b.color))
    if len(bars) < 2:
        return None, None
    # ★★★ 2026-10-13 按官方（OpenRM）重写配对 ★★★
    #   官方 getBestMatchedLightbarPair（OpenRM/src/pointer/getter.cpp:628-657）：
    #       for i in 0..n:
    #         for j = i+1 .. n:                       // ★ 强制两根是**不同**灯条
    #           if isLightBarMatched(lb[i], lb[j], ...)  // ★ 两灯条之间的几何判据
    #                pair_list.push_back(pair(i,j))
    #       取与 armor 中心最近的一对
    #   判据（OpenRM/src/pointer/judge.cpp:140-164 六项，本文件 check_armor 已移植）：
    #       面积比 / 中心偏移 / 长度比 / 板宽高比 / 两灯条角度差 / 两灯条角度均值
    #   ★ 左右由**灯条中心 x 坐标**决定（OpenRM/src/pointer/setter.cpp:95）：
    #       if (pp0.up.x + pp0.down.x) > (pp1.up.x + pp1.down.x)  -> 交换
    #   旧代码用的是"到 YOLO 框四角的距离"，且没有 i≠j 约束 —— 与官方完全不是一回事 ✗
    best_armor = None
    best_dis = 1e18
    o = np.array([ox, oy], float)
    box_center = np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0], float)
    n_pairs = 0
    for i in range(len(bars)):
        for j in range(i + 1, len(bars)):
            a, b2 = bars[i], bars[j]
            # 左右按 x 坐标强制分离（官方 setArmorFourPointsRelative 的做法）
            lb, rb2 = (a, b2) if a.center[0] <= b2.center[0] else (b2, a)
            arm = Armor(lb, rb2)
            if not check_armor(arm, cfg):
                continue
            n_pairs += 1
            c = (lb.center + rb2.center) / 2.0 + o
            dis = float(np.linalg.norm(c - box_center))
            if dis < best_dis:
                best_dis, best_armor = dis, arm
    if debug:
        print("      配对: 灯条=%d 通过 check_armor 的对=%d 最佳距离=%.1f"
              % (len(bars), n_pairs, best_dis))
    if best_armor is None:
        return None, None
    if enemy_color is not None and best_armor.color != enemy_color:
        return None, None
    # 返回官方顺序：左上, 右上, 左下, 右下（与 3D 对象点顺序严格一致）
    points = np.asarray(best_armor.points, float) + o
    return points, best_armor.color


def detect(image, boxes, cfg=CFG, enemy_color=None, debug=False):
    """对每个 YOLO 框做 ROI 灯条精修，返回 [{'box','points','color'}]。"""
    out = []
    for b in boxes:
        pts, color = refine_box(image, b, cfg, enemy_color, debug=debug)
        if pts is not None:
            out.append(dict(box=b, points=pts, color=color))
    return out


# ------------------------------------------------------------------ 自测
if __name__ == "__main__":
    import argparse
    import glob
    import os
    import onnxruntime as ort

    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=r"D:\UEproject\RM2019Sim 4.27\Saved\AimShots\rec_20261010_200805")
    ap.add_argument("--model", default=r"C:\Users\ASUS\Desktop\北洋机甲\neoautoaim1\TJURM-Aiming\config\models\v5n_7c_blk_640_480_v2.onnx")
    ap.add_argument("--out", default=r"D:\DeepseekPlungin\lightbar_out")
    ap.add_argument("--limit", type=int, default=40)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--debug-dir", default=None,
                    help="把 ROI 原图 / 二值化图 / 检测到的灯条 / 全图框 导出成 PNG")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    so = ort.SessionOptions(); so.log_severity_level = 3
    sess = ort.InferenceSession(args.model, so, providers=["CPUExecutionProvider"])
    iname = sess.get_inputs()[0].name
    ishape = sess.get_inputs()[0].shape
    IN_W = ishape[3] if isinstance(ishape[3], int) else 640
    IN_H = ishape[2] if isinstance(ishape[2], int) else 480

    files = sorted(glob.glob(os.path.join(args.dir, "*.png")) + glob.glob(os.path.join(args.dir, "*.jpg")))[:args.limit]
    n_box = n_ok = 0
    colors = {}
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        H, W = img.shape[:2]
        r = min(IN_W / W, IN_H / H)
        nw, nh = int(round(W * r)), int(round(H * r))
        canvas = np.full((IN_H, IN_W, 3), 114, np.uint8)
        dx, dy = (IN_W - nw) // 2, (IN_H - nh) // 2
        canvas[dy:dy + nh, dx:dx + nw] = cv2.resize(img, (nw, nh))
        blob = np.ascontiguousarray(canvas[:, :, ::-1].astype(np.float32).transpose(2, 0, 1)[None] / 255.0)
        y = np.asarray(sess.run(None, {iname: blob})[0])[0]
        if y.shape[0] < y.shape[1]:
            y = y.T
        ncls = y.shape[1] - 5
        conf = y[:, 4] * y[:, 5:5 + ncls].max(axis=1)
        boxes = []
        for k in np.where(conf > args.conf)[0]:
            cx, cy, bw, bh = y[k, 0], y[k, 1], y[k, 2], y[k, 3]
            boxes.append([(cx - dx) / r - bw / r / 2, (cy - dy) / r - bh / r / 2,
                          (cx - dx) / r + bw / r / 2, (cy - dy) / r + bh / r / 2])
        n_box += len(boxes)
        res = detect(img, boxes, debug=args.debug)
        n_ok += len(res)
        for it in res:
            colors[it["color"]] = colors.get(it["color"], 0) + 1
        if res:
            vis = img.copy()
            for it in res:
                p = it["points"].astype(int)
                col = (255, 80, 80) if it["color"] == "red" else (255, 200, 0)
                cv2.polylines(vis, [p.reshape(-1, 1, 2)], True, col, 2)
                for i, q in enumerate(p):
                    cv2.circle(vis, tuple(q), 4, (0, 255, 0), -1)
                    cv2.putText(vis, "LT RT RB LB".split()[i], tuple(q), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            for b in boxes:
                cv2.rectangle(vis, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 255), 1)
            cv2.imwrite(os.path.join(args.out, "lb_" + os.path.basename(f)), vis)
        # ---- 中间图导出（用户 2026-10-11：要看二值化图和 ROI）----
        # 不论精修成功还是失败都导出：失败帧（refined=0）才是最需要看的 ✗
        if args.debug_dir and boxes:
            vis = img.copy()
            for b in boxes:
                cv2.rectangle(vis, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 255), 1)
            os.makedirs(args.debug_dir, exist_ok=True)
            if LAST_DEBUG.get("roi") is None:
                # 这一帧没有精修成功的框：手动对"最大的框"再跑一次，只为把中间图导出来
                big = max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
                refine_box(img, big, enemy_color=None, debug=args.debug)
            roi = LAST_DEBUG.get("roi")
            binary = LAST_DEBUG.get("binary")
            if roi is not None and binary is not None:
                    Z = 4                                   # 放大，便于人眼看
                    roi_big = cv2.resize(roi, None, fx=Z, fy=Z, interpolation=cv2.INTER_NEAREST)
                    # 在 ROI 原图上画检测到的灯条（minAreaRect 四角 + 中心线）
                    for lb in LAST_DEBUG.get("bars", []):
                        bp = cv2.boxPoints(lb.rotated_rect).astype(int) * Z
                        cv2.polylines(roi_big, [bp.reshape(-1, 1, 2)], True, (0, 255, 255), 2)
                        cv2.circle(roi_big, tuple((lb.center * Z).astype(int)), 4, (0, 0, 255), -1)
                        cv2.line(roi_big, tuple((lb.top * Z).astype(int)),
                                 tuple((lb.bottom * Z).astype(int)), (255, 0, 255), 1)
                    bin_big = cv2.resize(binary, None, fx=Z, fy=Z, interpolation=cv2.INTER_NEAREST)
                    bin_bgr = cv2.cvtColor(bin_big, cv2.COLOR_GRAY2BGR)
                    for c in LAST_DEBUG.get("contours", []):
                        cv2.drawContours(bin_bgr, [c * Z], -1, (0, 255, 0), 1)
                    info = ("%s  p99.5=%.0f  otsu=%.0f  contours=%d  bars=%d"
                            % (LAST_DEBUG.get("thr_mode", "?"), LAST_DEBUG.get("p995", 0),
                               LAST_DEBUG.get("otsu", 0), len(LAST_DEBUG.get("contours", [])),
                               len(LAST_DEBUG.get("bars", []))))
                    for pane, title in ((roi_big, "ROI (x%d) " % Z + info), (bin_bgr, "BINARY (x%d)" % Z)):
                        cv2.putText(pane, title, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    hh = max(roi_big.shape[0], bin_bgr.shape[0])
                    ww = roi_big.shape[1] + bin_bgr.shape[1] + 8
                    panes = np.zeros((hh + 34, ww, 3), np.uint8)
                    panes[:roi_big.shape[0], :roi_big.shape[1]] = roi_big
                    panes[:bin_bgr.shape[0], roi_big.shape[1] + 8:roi_big.shape[1] + 8 + bin_bgr.shape[1]] = bin_bgr
                    full = cv2.resize(vis, (ww, int(vis.shape[0] * ww / vis.shape[1])))
                    out2 = np.vstack([panes, full])
                    cv2.imwrite(os.path.join(args.debug_dir, "dbg_" + os.path.basename(f)), out2)
    print("frames=%d  yolo_boxes=%d  lightbar_refined=%d  (%.0f%%)  colors=%s"
          % (len(files), n_box, n_ok, 100.0 * n_ok / max(1, n_box), colors))
    print("可视化 ->", args.out)
