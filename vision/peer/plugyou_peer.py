# plugyou_peer.py —— 把 PlugYourRM 的自瞄方案接到 UE 仿真上（用户 2026-10-11："接"）
#
# 分工（对照 PlugYourRM/docs/architecture.md 的"sim / custom 两层由总线相遇"）：
#   我们**替掉 sim 那一侧**，只保留它的两个通信边界：
#       输入：DetectionFrame（只有 2D 图像点，LT/RT/RB/LB）+ SelfState（世界系位姿 + 内参 K）
#       输出：ShotSolution（绝对云台角、pitch 抬头为正、fire 推荐）
#   中间整条链是**他们自己的代码**：
#       pose_solver("tjurm") → tracker("multi_target", 内含 EKF 卡尔曼 + 状态机 + 目标池)
#                            → armor_selector(选板) → traj_solver(弹道) → shooter(开火门控)
#
# 视觉前端用的是**他们真车的传统灯条识别**（YOLO 出框 → ROI 内找灯条 → 配对 → 四角点 + 红蓝），
# 见 Tools/wl_lightbar.py（1:1 移植自 sp_vision_25-main/tasks/auto_aim/detector.cpp）。
#
# 协议（和 Source/RM2019Sim/WLAimNet.h 一致，localhost）：
#   41001 收帧：  uint32 'RMAI' uint32 FrameId uint16 ChunkIdx uint16 ChunkCount uint32 Total uint32 Payload + JPEG
#   41002 收状态：uint32 'RMST' float gimbalYawDeg float gimbalPitchDeg float rollDeg float timeSec
#                 + [可选 48 字节] 6 个世界系 float3：chassis_pos, chassis_rpy, gimbal_pos, gimbal_rpy,
#                   muzzle_pos, camera_rpy   ← 用于构造他们的 SelfState
#   41003 发指令：uint32 'RMCM' float dYawDeg float dPitchDeg uint8 fire uint8 has_target 2x pad float timeSec
#
# 用法：
#   python -u plugyou_peer.py [--sweep 0] [--no-lightbar] [--dry DIR]

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np
import onnxruntime as ort

# ---- PlugYourRM 的代码路径（本地镜像）----
PLUGYOU = r"D:\PlugYourRM"
if PLUGYOU not in sys.path:
    sys.path.insert(0, PLUGYOU)

TOOLS = os.path.dirname(os.path.abspath(__file__))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

MAGIC_FRAME = 0x49414D52      # 'RMAI'
MAGIC_STATE = 0x54534D52      # 'RMST'
MAGIC_CMD = 0x4D434D52        # 'RMCM'

# 我们的相机内参（1280x1024 + 6mm -> f=1250px，cx/cy 为半分辨率中心，无畸变）
CAM_W, CAM_H, F_PX = 1280, 1024, 1250.0
CAM_GIMBAL_OFF = 0.05      # 相机在云台前方的安装距离（对齐他们 data.yaml 的 camera_mount_pos ✓）
# ★ 2026-10-12：板尺寸必须与 PnP 模型一致。
#   桥接层调 estimate(armor_size="small")，而 plate_size("small") 从他们的 data.yaml 取
#   light_bar_interval=0.135 / light_bar_length=0.055。以前这里写 0.13x0.13（正方形），
#   与模型不符 ⇒ 测距偏 +40%~+100%（peer.log 里 dist=0.36 那类畸形值就是这么来的）。
#   这里只用于 yaw_pnp 那条可选通路；PnP 本身已由 armor_size 走他们的 config ✓。
PLATE_W = 0.135            # 与 data.yaml 的 light_bar_interval 对齐（不是仿真渲染尺寸）
PLATE_H = 0.055            # 与 data.yaml 的 light_bar_length 对齐
ARMOR_TILT_DEG = 0.0       # 仿真的板是竖直的 ✓（他们真车是 15 度）
YAW_PNP = False
SHARED = {'px_err_v': None, 'px_err': None}
TF_FILT = {'yaw': None, 'pitch': None}
YAW_BIAS = [0.0]   # yaw ?????????????? ??
HOLD = {'yaw': None, 'pitch': None, 'dist': None}   # ????????????? ?   # yaw/pitch ??????? ?   # main ? keeper ?????? ?
G_SOLVER = [None]    # ?? ????????????? global ??
PITCH_BIAS = [0.0]
_LIVE_LEN = 0            # 当前"原地刷新"行的可见长度（用于右侧补空格擦除残字）
ADJUST = {'tag': '', 't': 0.0}   # 最近一次按键调整（由主循环的 _live 行统一显示）
# 相机 -> 枪口 偏移（仿真世界系，米），从帧包里解出来；用于把瞄准线从"相机→目标"
# 换成"枪口→目标"。没有该字段时保持 None（老仿真）⇒ 退回相机→目标。
DOFTRIM = {'yaw': 0.0, 'pitch': 0.0}
# ★★ 自动配平（2026-10-12）★★
#   现场表现：稳定跟踪一块正对的板子时，炮口相对板心有一个**恒定**偏置
#   （用户实测"偏右上"）。与其人工试 trim 的符号和大小，不如让程序自己量：
#   跟踪稳定时 vision 给的 yaw/pitch 就是"炮口相对目标的残余偏差"（正=目标在左），
#   取一段时间的均值再取反加回去 ⇒ 弹着点自动落到板心 ✓
#   触发：控制台敲 `bias`（回车）开始采样，约 2 秒后自动生效；敲 `bias off` 清除。
AUTOBIAS = {
    'collect': False,      # 正在采样？
    'n': 0,                # 样本数
    'sy': 0.0, 'sp': 0.0,  # yaw/pitch 累加
    'yaw': 0.0, 'pitch': 0.0,   # 已生效的偏置
    'active': False,
}

# ★★ 2026-10-12：稳态误差积分器（消除"从不同方向过来停在不同的地方"）★★
#   现象：两车都不动，但从左边跟过去停在一个角度、从右边跟过去停在另一个角度。
#   原因：整条回路是**纯比例**控制（peer: 误差×增益；仿真: Rate += -MouseYawRateCmd）
#         ⇒ 有摩擦/死区时，误差小到推不动就停下，残余误差的方向取决于从哪边接近 ✓
#   修法：在 peer 侧对"角度残差"做慢积分，把小残差累积成足够的推动量，最终把稳态误差压到 0。
#        只在**像素误差很小**（确实锁定目标）时才积分，避免检测抖动被积成大偏移 ✓
INTEG = {
    'yaw': 0.0, 'pitch': 0.0,
    'lim': 3.0,        # 积分项限幅（度）—— 最多额外补 3 度，防止跑飞
    'rate': 0.02,      # 每秒积分速率（度/秒 per 度误差）
    'pxgate': 90.0,    # 只有 |px_err| 小于这个值才积分（像素）
}
BIAS_ADAPT = False         # 2026-10-12：TF 改成精确解后停用自学习偏置（见 _keeper 的说明）；要恢复设 True
PITCH_ABS_MODE = [True]   # True = ??? target_pitch ???????? ???? yaw ??/pitch ?? ??      # ??????? pitch ????????

def _Rz(a):
    c, sa = math.cos(a), math.sin(a)
    return np.array([[c, -sa, 0.0], [sa, c, 0.0], [0.0, 0.0, 1.0]])


def _live(text):
    """不换行地覆盖控制台**最后一行**（原地刷新）。

    三条铁律（踩过的坑，别再改）：
      1) **不用 ANSI 转义**：cmd.exe 默认没开虚拟终端，\\033[K 会被当普通字节打出来 ✗
      2) **右侧补空格**擦掉上一行的残字（旧行更长时残字会把行撑到换行 ⇒ 看着像刷屏 ✗）
      3) **留出 1 列**：写满整行时终端的自动换行会让 \\r 回不到行首 ⇒ 每写满一次多滚一行 ✗
         （这一条就是"日志一直刷"的真正原因：主循环和调偏置线程各写一行，其中一行满列）
    另外：所有周期性输出都必须走这里，**不要有两套写入者**。"""
    global _LIVE_LEN
    try:
        text = text.encode("gbk", "replace").decode("gbk")   # cmd 默认 GBK，先换算真实列宽
    except Exception:
        pass
    cols = _term_cols() - 1                  # 留 1 列，避免自动换行
    if len(text) > cols:
        text = text[:cols]
    pad = max(0, _LIVE_LEN - len(text))
    sys.stdout.write("\r" + text + " " * pad)
    sys.stdout.flush()
    _LIVE_LEN = len(text)


def _term_cols():
    """终端宽度；拿不到就退回默认值。"""
    try:
        return int(os.environ.get("COLUMNS") or 0) or _shutil_cols()
    except Exception:
        return 120


def _shutil_cols():
    try:
        import shutil
        return shutil.get_terminal_size((120, 30)).columns
    except Exception:
        return 120


def _live_done():
    """结束原地刷新，换行（打正式日志前调用，避免日志和状态行粘在一起）。"""
    global _LIVE_LEN
    if _LIVE_LEN:
        sys.stdout.write("\n")
        sys.stdout.flush()
        _LIVE_LEN = 0


def FMathUnwind(deg):
    """角度归一到 (-180, 180]，与仿真里的 FMath::UnwindDegrees 同义（诊断日志用）。"""
    while deg > 180.0:
        deg -= 360.0
    while deg <= -180.0:
        deg += 360.0
    return deg


# ====================================================================== 弹道补偿
# 为什么必须做（2026-10-12 用户实测"离远了 pitch 不往上抬 / 打不中"）：
#   仿真 WLPellet.h:4-35 —— 17 mm 凝胶弹、3.2 g、**初速 23 m/s**、**二次空气阻力**、g=9.81。
#   23 m/s 下坠很凶：3 m 要抬 1.66°、6 m 要抬 3.46°、10 m 要抬 6.08°、15 m 要抬 9.75°。
#   而 peer 之前**完全没做补偿**（距离字段甚至没被解析）⇒ 近距靠运气，远距必掉 ✓
#   注：视觉端 data.yaml 配的是 speed 25 / 无阻力，与仿真不一致，所以这里按**仿真参数**算。
BALLISTIC = {
    'enable': True,
    'v0': 23.0,        # m/s，仿真 MuzzleSpeedCmS = 2300
    'cd': 0.47,        # 球体绕流
    'dia': 0.017,      # m
    'mass': 0.0032,    # kg
    'rho': 1.2,        # kg/m^3
    'g': 9.81,
    'cache': {},       # range(m,1位小数) -> (superelev_deg, flight_s)
}


def _ballistic_k():
    import math as _m
    A = _m.pi * (BALLISTIC['dia'] / 2.0) ** 2
    return 0.5 * BALLISTIC['rho'] * BALLISTIC['cd'] * A / BALLISTIC['mass']


def ballistic_superelevation(rng_m):
    """水平打中距离 rng_m 的目标，炮口需要额外抬高的角度（度）。带缓存。

    解 x'' = -K v x',  y'' = -g - K v y'（K = 0.5 rho Cd A / m，单位 1/m）。
    积分步长自适应：远距离用大步长即可（我们只要角度，精度 0.01° 足够）。
    """
    import math as _m
    if not BALLISTIC['enable'] or rng_m is None or rng_m <= 0.05:
        return 0.0, 0.0
    key = round(float(rng_m), 1)
    hit = BALLISTIC['cache'].get(key)
    if hit is not None:
        return hit
    K = _ballistic_k()
    v0, g = BALLISTIC['v0'], BALLISTIC['g']
    x = y = 0.0
    vx, vy = v0, 0.0
    dt = 2e-4 if rng_m < 6.0 else 1e-3
    t = 0.0
    while x < rng_m and t < 5.0:
        v = _m.hypot(vx, vy)
        vx += (-K * v * vx) * dt
        vy += (-g - K * v * vy) * dt
        x += vx * dt
        y += vy * dt
        t += dt
    drop = -y
    ang = _m.degrees(_m.atan2(drop, x if x > 1e-6 else rng_m))
    if len(BALLISTIC['cache']) > 512:
        BALLISTIC['cache'].clear()
    BALLISTIC['cache'][key] = (ang, t)
    return ang, t


def _win_focus():
    """把本进程的控制台窗口拉到前台，确保 msvcrt 能收到方向键。

    为什么需要（2026-10-12 实测）：本进程由 launcher 用 CREATE_NEW_CONSOLE 起，
    但窗口可能不在前台 ⇒ getch() 收不到任何键 ⇒ 用户"按了方向键没反应" ✓
    """
    try:
        import ctypes
        k = ctypes.windll.kernel32
        hwnd = k.GetConsoleWindow()
        if hwnd:
            ctypes.windll.user32.ShowWindow(hwnd, 9)          # SW_RESTORE
            ctypes.windll.user32.SetForegroundWindow(hwnd)
    except Exception:
        pass


def _start_bias_console():
    """现场调偏置窗口（热更新）。

    为什么做成独立窗口 + 方向键：改偏置必须**立刻看到效果**，重启 peer 一次 3 秒、
    还要重新等目标锁上，试符号/量级时太慢。这里直接改进程内的变量，keeper 下一帧就用上 ✓

    按键（在 peer 的窗口里）：
        ←  →     yaw 偏置 −/+0.1°（正 = 往左补）
        ↑  ↓     pitch 偏置 +/−0.1°（正 = 往抬头补）
        PageUp/Down  同上，步长 ×10（1.0°）
        b        自动配平：采 ~1.5s 均值，自动算出该补多少（推荐先用这个）
        r        全部清零
        ?        打印帮助
    """
    import threading

    def loop():
        # Windows 下用 msvcrt 读键（带方向键）；没有就退回按行输入
        try:
            import msvcrt
            _win_focus()
        except Exception:
            msvcrt = None

        if msvcrt is not None:
            print("[bias] 键盘已接管。若按键无反应，先用鼠标点一下本窗口标题栏再按 ✓")
            while True:
                try:
                    ch = msvcrt.getch()
                except Exception:
                    return
                if ch in (b"\x00", b"\xe0"):          # 扩展键前缀
                    try:
                        ch2 = msvcrt.getch()
                    except Exception:
                        return
                    step = 1.0 if False else 0.1
                    if ch2 == b"K":                    # ←
                        AUTOBIAS['yaw'] -= step; _show("yaw-")
                    elif ch2 == b"M":                  # →
                        AUTOBIAS['yaw'] += step; _show("yaw+")
                    elif ch2 == b"H":                  # ↑
                        AUTOBIAS['pitch'] += step; _show("pitch+")
                    elif ch2 == b"P":                  # ↓
                        AUTOBIAS['pitch'] -= step; _show("pitch-")
                    elif ch2 == b"I":                  # PageUp
                        AUTOBIAS['pitch'] += 1.0; _show("pitch+1")
                    elif ch2 == b"Q":                  # PageDown
                        AUTOBIAS['pitch'] -= 1.0; _show("pitch-1")
                    AUTOBIAS['active'] = True
                    continue
                c = ch.decode("latin1").lower()
                if c == "b":
                    _live_done()
                    AUTOBIAS.update(collect=True, n=0, sy=0.0, sp=0.0)
                    print("[bias] 采集中… 保持瞄准那块正对的板子别动（约 1.5 s）")
                elif c == "r":
                    _live_done()
                    AUTOBIAS.update(collect=False, n=0, sy=0.0, sp=0.0,
                                    yaw=0.0, pitch=0.0, active=False)
                    INTEG['yaw'] = 0.0
                    INTEG['pitch'] = 0.0
                    _show("cleared")
                elif c == "?":
                    _live_done()
                    _help()
                elif c == "y":
                    _persist()
                else:
                    # ★ 任何**未知按键**都回显一次：用于确认"键盘到底有没有进来" ✓
                    _live_done()
                    print("[bias] 收到按键 %r（键盘有效）；当前 yaw=%+.2f pitch=%+.2f"
                          % (ch, AUTOBIAS['yaw'], AUTOBIAS['pitch']))
        else:
            while True:
                try:
                    line = sys.stdin.readline()
                except Exception:
                    return
                if not line:
                    return
                cmd = line.strip().lower()
                if cmd == "bias":
                    AUTOBIAS.update(collect=True, n=0, sy=0.0, sp=0.0)
                    print("[bias] collecting...")
                elif cmd in ("bias off", "r"):
                    AUTOBIAS.update(collect=False, n=0, sy=0.0, sp=0.0,
                                    yaw=0.0, pitch=0.0, active=False)
                    _show("cleared")
                elif cmd.startswith("yaw"):
                    try:
                        AUTOBIAS['yaw'] = float(cmd.split()[1]); AUTOBIAS['active'] = True
                    except Exception:
                        pass
                    _show("yaw set")
                elif cmd.startswith("pitch"):
                    try:
                        AUTOBIAS['pitch'] = float(cmd.split()[1]); AUTOBIAS['active'] = True
                    except Exception:
                        pass
                    _show("pitch set")
                elif cmd == "?":
                    _help()

    def _help():
        print("""
[bias help]  正 yaw = 往左补   正 pitch = 往抬头补
  ← →         yaw  -/+ 0.1
  ↑ ↓         pitch +/-
  PgUp PgDn   pitch +/- 1.0
  b           自动配平（推荐）
  r           清零
  y           把当前值写进 bias_saved.json（下次启动自动加载）
  ?           本帮助
""")

    def _show(tag):
        # ★ 2026-10-12：**不再单独打印**。调偏置的线程只做两件事：改值 + 记下"刚调了什么"，
        #   显示交给主循环唯一那条 _live 状态行 ⇒ 全局只有**一个**写入者，绝不互相覆盖 ✓
        #   （原来这里自己写一行、主循环又写一行，其中一行写满列宽就多滚一行 ✗）
        ADJUST['tag'] = tag
        ADJUST['t'] = time.time()

    def _persist():
        try:
            import json
            p = os.path.join(TOOLS, "bias_saved.json")
            with open(p, "w", encoding="utf-8") as fh:
                json.dump({'yaw': AUTOBIAS['yaw'], 'pitch': AUTOBIAS['pitch']}, fh)
            print("\n[bias] saved -> %s" % p)
        except Exception as e:
            print("\n[bias] save failed: %s" % e)

    threading.Thread(target=loop, daemon=True).start()

def _Ry(a):
    c, sa = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, sa], [0.0, 1.0, 0.0], [-sa, 0.0, c]])      # ??? PnP ????qcqp??? ?            # ★ 暂不采用移植版的 yaw（verify_yawpnp.py 证明它在部分几何上会卡搜索边界 ✗）；
                           #   仅保留 [yawpnp] 对照日志 ✓。等它跑到 15/15 再打开 ✓。
                           #   实测：单板 + 绕向归一化之后**他们自己的 yaw 已经稳定** ✓（恒定 -36.1° ✓）
K = np.array([[F_PX, 0.0, 0.0],
              [0.0, F_PX, 0.0],
              [0.0, 0.0, 1.0]], float)   # 主点 (0,0)：他们文档 §5.1，像素以图像中心为原点


# ====================================================================== 视觉
class VisionFront:
    """YOLO 出框 → （可选）传统灯条精修 → Detection 列表。"""

    # ★★ 2026-10-12：置信度阈值 0.25 -> 0.08 ★★
    #   实测（当前曝光下的一张真实帧，直接跑模型看原始输出）：
    #       最高 conf = 0.165（cls=4，就是装甲板），其余 0.098 / 0.077 / 0.059 ...
    #   而阈值是 0.25 ⇒ **模型明明检测到了，却被全部滤掉** ⇒ boxes=0 ⇒
    #     peer 不发检测回传 ⇒ 仿真的 DetFrameId 变旧 ⇒ HUD 不画绿框 ⇒ 用户"绿框没了" ✓
    #   这张图 mean 亮度只有 0.5、max 145（曝光压得极暗），置信度天然低。
    #   低阈值靠后面的 _plausible_armor（宽 20..400、长宽比 1.2..5）+ 灯条精修把关 ✓
    def __init__(self, model, use_lightbar=True, conf=0.08, enemy_color=None, box_fallback=True):
        import wl_lightbar
        self.lb = wl_lightbar
        self.use_lightbar = use_lightbar
        self.box_fallback = box_fallback
        self.conf = conf
        self.enemy_color = enemy_color
        so = ort.SessionOptions(); so.log_severity_level = 3
        # ★ 2026-10-13 用户要求换回 GPU（绿框容差已改 8 帧，闪的问题已修）
        #   CUDA 需要能找到 pip 装的 cuDNN/cuBLAS ⇒ 先把它们的 bin 加入 DLL 搜索路径
        try:
            import site as _site
            for _sp in _site.getsitepackages():
                for _sub in ("nvidia/cudnn/bin", "nvidia/cublas/bin", "nvidia/cuda_nvrtc/bin"):
                    _d = os.path.join(_sp, *_sub.split("/"))
                    if os.path.isdir(_d):
                        os.environ["PATH"] = _d + os.pathsep + os.environ["PATH"]
                        try:
                            os.add_dll_directory(_d)
                        except Exception:
                            pass
        except Exception as _e:
            print("[ai] dll path setup failed: %s" % _e)
        try:
            _avail = ort.get_available_providers()
        except Exception:
            _avail = []
        _pref = [_q for _q in ("CUDAExecutionProvider",) if _q in _avail] + ["CPUExecutionProvider"]
        try:
            self.sess = ort.InferenceSession(model, so, providers=_pref)
        except Exception as _e:
            print("[ai] GPU failed (%s) -> CPU" % _e)
            self.sess = ort.InferenceSession(model, so, providers=["CPUExecutionProvider"])
        print("[ai] onnxruntime providers = %s" % self.sess.get_providers())
        self.iname = self.sess.get_inputs()[0].name
        sh = self.sess.get_inputs()[0].shape
        self.iw = sh[3] if isinstance(sh[3], int) else 640
        self.ih = sh[2] if isinstance(sh[2], int) else 480

    def _yolo(self, img):
        H, W = img.shape[:2]
        r = min(self.iw / W, self.ih / H)
        nw, nh = int(round(W * r)), int(round(H * r))
        canvas = np.full((self.ih, self.iw, 3), 114, np.uint8)
        dx, dy = (self.iw - nw) // 2, (self.ih - nh) // 2
        canvas[dy:dy + nh, dx:dx + nw] = cv2.resize(img, (nw, nh))
        blob = np.ascontiguousarray(canvas[:, :, ::-1].astype(np.float32).transpose(2, 0, 1)[None] / 255.0)
        y = np.asarray(self.sess.run(None, {self.iname: blob})[0])[0]
        if y.shape[0] < y.shape[1]:
            y = y.T
        ncls = y.shape[1] - 5
        conf = y[:, 4] * y[:, 5:5 + ncls].max(axis=1)
        clsid = y[:, 5:5 + ncls].argmax(axis=1)
        out = []
        for k in np.where(conf > self.conf)[0]:
            cx, cy, bw, bh = y[k, 0], y[k, 1], y[k, 2], y[k, 3]
            out.append(([(cx - dx) / r - bw / r / 2, (cy - dy) / r - bh / r / 2,
                         (cx - dx) / r + bw / r / 2, (cy - dy) / r + bh / r / 2],
                        float(conf[k]), int(clsid[k])))
        return out

    @staticmethod
    def _nms(boxes, iou_thr=0.35):
        """同一块装甲板常被模型输出成多个重叠框（实测 plates=3 其实是一块板 ✗）。
        他们的 tracker 要求"同一目标连续 min_detect_count 帧"才锁定，喂 3 个重复目标会永远锁不住
        （实测表现：yaw/pitch 恒定不动、fire 恒 False）。所以按 IoU 去重，一块板只留最高分的那个框。"""
        keep = []
        for b, c, k in sorted(boxes, key=lambda z: -z[1]):
            x1, y1, x2, y2 = b
            dup = False
            for kb, kc, kk in keep:
                a1, b1, a2, b2 = kb
                ix1, iy1 = max(x1, a1), max(y1, b1)
                ix2, iy2 = min(x2, a2), min(y2, b2)
                iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
                inter = iw * ih
                ua = (x2 - x1) * (y2 - y1) + (a2 - a1) * (b2 - b1) - inter
                if ua > 0 and inter / ua > iou_thr:
                    dup = True
                    break
            if not dup:
                keep.append((b, c, k))
        return keep
    @staticmethod
    def _canonical_pts(pts):
        """把四角点归一到**固定绕向**（图像坐标下逆时针）。
        为什么（2026-10-12 用户实测"单板静止靶 yaw 也乱甩" ✓）：4 点共面的 PnP 有两个镜像解 ✗，
        角点绕向一翻，解出的板法线就翻 180 度 ⇒ 整车 yaw 跳几十度 ⇒ 输出乱甩 ✓✓。
        灯条左右配对在"板正对相机"时最容易左右抖 ✓ ⇒ 这里按面积符号强制绕向 ✓，
        并让 LT 永远是"较小的那对 x 里的上点" ⇒ 顺序逐帧稳定 ✓。
        """
        # ★ 先整体 reshape 成 (4,2) —— 实测 pts 有时是**扁平 (8,)** ✗，
        #   直接逐元素 reshape(2) 会得到 8 个标量 ⇒ 返回一维数组 ⇒ 后面 reshape(4,2) 崩 ✓
        arr = np.asarray(pts, float).reshape(-1)
        if arr.size != 8:
            return np.asarray(pts, float).reshape(4, 2)
        q = [arr[i * 2:i * 2 + 2].copy() for i in range(4)]
        # 面积符号（图像 y 向下：顺时针为负 ✓）
        area = 0.0
        for i in range(4):
            x1, y1 = q[i]
            x2, y2 = q[(i + 1) % 4]
            area += x1 * y2 - x2 * y1
        if area < 0:                        # 统一成逆时针（面积 > 0）
            q = [q[0], q[3], q[2], q[1]]
        # 再按 "x 最小者在上" 固定起点（避免整体旋转一个相位 ✓）
        xs = [v[0] for v in q]
        i0 = int(np.argmin(xs))
        if q[i0][1] > q[(i0 + 3) % 4][1]:
            q = [q[(i0 + 3) % 4]] + q[i0:(i0 + 3)]
        elif i0 != 0:
            q = q[i0:] + q[:i0]
        return np.array(q, float)

    @staticmethod
    def _plausible_armor(box):
        """只留下"像装甲板"的框。
        实测（2026-10-11）：模型每帧给 10~12 个框，里面混着车体/墙等**非装甲板**目标 ✗；
        全喂给他们的管线会把 tracker 淹没 ⇒ 表现是 sol 恒 0、永不锁定 ✓✓。
        装甲板在画面里的形态：宽 20~400 px、长宽比 1.2~5（正视偏扁）。"""
        x1, y1, x2, y2 = box
        w, h = (x2 - x1), (y2 - y1)
        if h < 8 or w < 20:
            return False
        ar = w / max(1e-3, h)
        return (1.2 <= ar <= 5.0) and (20.0 <= w <= 400.0)

    def detect(self, jpg_bytes):
        img = cv2.imdecode(np.frombuffer(jpg_bytes, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return None, 0, 0
        raw = self._yolo(img)
        boxes = self._nms([(b, c, k) for (b, c, k) in raw if self._plausible_armor(b)])
        # ★ 只保留**一块板**（离画面中心最近的那个）—— 2026-10-12 用户实测"静止靶 yaw 乱甩" ✓
        # 原因：一帧喂多块板时，他们的 EKF（armor_count=4 的整车模型）要在"这是哪块板"之间
        # 做 ID 匹配 ⇒ 匹配一抖，整车 yaw 估计就跳几十度 ⇒ 输出 yaw 乱甩 ✓✓。
        # 单板输入下不存在这个歧义 ✓（也是他们 standard 单板自瞄的常规用法 ✓）。
        # ★★★ 2026-10-13 用户报"小陀螺（靶车自转）时跟随有问题、灯条识别乱跳" ★★★
        #   根因：自转时**两块装甲板会交替经过画面中心**，而这里的规则是"取离中心最近的那块"
        #   ⇒ 两块板轮流中选 ⇒ 解出来的角在两者之间来回跳（差远大于两块板的实际夹角）✗
        #   修法：**时间关联 + 迟滞** —— 优先选"离上一帧选中的那块最近"的板，
        #         只有跟丢了（离得太远）才回到"离画面中心最近" ✓
        if len(boxes) > 1:
            def _ctr(b):
                x1, y1, x2, y2 = b[0]
                return ((x1 + x2) / 2.0 - CAM_W / 2.0) ** 2 + ((y1 + y2) / 2.0 - CAM_H / 2.0) ** 2

            def _cxy(b):
                x1, y1, x2, y2 = b[0]
                return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

            _prev = SHARED.get('last_plate_ctr')
            _picked = None
            if _prev is not None and (time.time() - _prev[2]) < 0.5:
                def _d2prev(b):
                    cx, cy = _cxy(b)
                    return (cx - _prev[0]) ** 2 + (cy - _prev[1]) ** 2
                _cand = min(boxes, key=_d2prev)
                _cx, _cy = _cxy(_cand)
                if math.hypot(_cx - _prev[0], _cy - _prev[1]) <= 0.25 * CAM_W:
                    _picked = _cand          # 与上一帧的板对上了 ⇒ 保持同一块 ✓
            if _picked is None:
                _picked = min(boxes, key=_ctr)   # 跟丢了 ⇒ 回到"离中心最近"
            boxes = [_picked]
            _cx, _cy = _cxy(_picked)
            SHARED['last_plate_ctr'] = (_cx, _cy, time.time())
        dets = []
        n_refined = 0
        self.last_classes = [k for (b, c, k) in boxes]
        for box, cf, kid in boxes:
            pts, color = (None, None)
            if self.use_lightbar:
                pts, color = self.lb.refine_box(img, box, enemy_color=self.enemy_color)
            if pts is not None:
                n_refined += 1
                # [REVERT 2026-10-12] ??????? ??????????????? LT/RT/RB/LB ??
                #   ????????????? ? ??/??/PnP ??? ????????????????? ??
                dets.append((pts, color, cf))
            elif self.box_fallback:
                # 兜底：只对**已通过装甲板形态门限**的框用框角点（此时框本身就是板，四角够用 ✓）。
                # 绝不对未过滤的框这么做 —— 那会让他们的 PnP 解出垃圾位姿（实测把炮管压到最低 ✗）。
                x1, y1, x2, y2 = box
                pts = np.array([(x1, y1), (x2, y1), (x2, y2), (x1, y2)], float)
                dets.append((pts, None, cf))
        return dets, len(boxes), n_refined


# ====================================================================== 自车状态
class SelfStateBuilder:
    """把仿真发来的位姿组装成他们的 SelfState。

    契约（core/bus/dataflow/self_state.py）：所有 pos 都是**世界系绝对坐标**，
    K 是 3x3 相机内参。我们是理想相机：camera.pos = muzzle.pos，camera.rpy = gimbal.rpy。
    """

    def __init__(self):
        self.raw = None

    def update(self, vals):
        self.raw = vals

    def build(self, t):
        from core.bus.dataflow import SelfState, RigidState, CameraState
        if self.raw is None:
            return None
        (gy, gp, roll, ts, cpos, crpy, gpos, grpy, mpos, camrpy) = self.raw
        # 单位换算：仿真内部是**厘米 + 度**，他们的契约是**米 + 弧度**（见 self_state.py 与
        # armor.hpp 里的 ypr_in_gimbal "单位：rad"）。位置也换算成米。
        def pos_cm_to_m(v):
            # UE 是 x前/y右/z上（左手系），他们是 x前/y左/z上（右手系）=> y 取负
            a = np.array(v, float) / 100.0
            return np.array([a[0], -a[1], a[2]], float)

        def deg_to_rad(v):
            # 仿真现在直接在包里给**仿真世界系**的 (roll, pitch, yaw)（度），和他们约定一致：
            #   正 yaw = 向左（逆时针）✓、正 pitch = 抬头 ✓
            # 所以这里**只做单位换算**：以前那套 [-r, p, -y] 的 UE 镜像修正必须去掉 ✗
            # （留着它就会把已经正确的世界系角再翻一次 ✗）。
            return np.deg2rad(np.array(v, float))

        zero = np.zeros(3)
        # ★ 关键修正：**云台朝向用相机朝向**，不要用仿真里那个 "gimbal rpy" 字段 ✗
        # 这台车的骨骼系是 Blender 那套，`PitchX.Rotator()` 不是云台的世界朝向
        # （踩过：拿它当相机朝向会直接看天 ✗✗）。相机刚性装在云台上 ⇒ 两者朝向相同 ✓。
        # 真值标定（gt_calib.py）已证明：喂对了他们的解算误差只有 1~2.4° ✓，
        # 而现场那个"pitch 压到底 −57°"只能是这里喂错导致的 ✓。
        rpy_used = deg_to_rad(camrpy)
        # ★★ gimbal.pos 按**他们的安装约定**反推（2026-10-11 审计结论）★★
        # 他们的 pose_solver L40 用 camera_pos_gimbal = cam.pos − gimbal.pos 做 C'→G 平移，
        # 而 data.yaml 里这个安装偏置是固定的： small 机器人 gimbal_mount_pos=[0,0,0]、
        # camera_mount_pos=[0.05,0,0] ⇒ 相机在云台前方 **5 cm**。
        # 仿真里那个 "gimbal pos" 字段给的是 PitchX 骨骼位置（俯仰轴，未必等于 yaw 轴原点 ✗），
        # frame=G 下它就是**目标坐标的基准点** ⇒ 基准错了目标位置整体偏 ⇒ 方位/仰角都错 ✓。
        # 所以这里直接用 [cam.pos − R_cam·(0.05,0,0)] 作为云台中心 ✓，与他们的假设严格一致 ✓。
        # [FIX 2026-10-12] gimbal.pos ???**??????????**???????? ?
        #   ??? pose_solver L40 ? (cam.pos - gimbal.pos) ??"G ?(???)????????" ??
        #   G ???????? ??????? R_cam??????????? ? ?????????
        #   ? **?/???????????????? yaw/pitch ?????** ????????? ???
        cam_pos = pos_cm_to_m(mpos)
        gimbal_pos = pos_cm_to_m(gpos)
        return SelfState(
            chassis=RigidState(pos=pos_cm_to_m(cpos), rpy=deg_to_rad(crpy), vel=zero.copy(), omg=zero.copy()),
            gimbal=RigidState(pos=gimbal_pos, rpy=rpy_used, vel=zero.copy(), omg=zero.copy()),
            muzzle=RigidState(pos=pos_cm_to_m(mpos), rpy=rpy_used, vel=zero.copy(), omg=zero.copy()),
            camera=CameraState(pos=cam_pos, rpy=rpy_used, K=K.copy()),
        )


# ====================================================================== UDP
class UdpBridge:
    def __init__(self):
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        self.rx.bind(("127.0.0.1", 41001))
        self.rx.setblocking(False)
        self.st = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.st.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.st.bind(("127.0.0.1", 41002))
        self.st.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dst = ("127.0.0.1", 41003)
        # 41004：把检测结果回传给仿真，让它画在自瞄画面上（用户要看得到识别结果）
        self.dtx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ddst = ("127.0.0.1", 41004)
        self.frames = {}
        self.frame_pose_map = {}   # ★ fid -> (9 float 位姿)：位姿与图像**严格同帧**（2026-10-12 修复）
        self.last_id = -1
        self.state_raw = None
        # 帧内携带的相机位姿（仿真 2026-10-12 起）：(Px,Py,Pz, Yaw,Pitch,Roll)，米/度
        self.frame_pose = None
        self.frame_pose_id = -1
        self.range_cm = None          # 状态包 offset20：到目标的距离(cm)，用于弹道补偿
        self.n_frames = 0
        self.last_frame_id = 0
        self.n_cmds = 0
        self.n_abs = 0
        self.n_inc = 0
        self.last_cmd = (0.0, 0.0, False)
        # 指令与开火，供 keeper 线程取
        self.lock = threading.Lock()

    def pump(self):
        got = None
        while True:
            try:
                data, _ = self.rx.recvfrom(2048)
            except (BlockingIOError, OSError):
                break
            if len(data) < 20:
                continue
            magic, fid, cidx, ccnt, total, plen = struct.unpack_from("<IIHHII", data, 0)
            if magic != MAGIC_FRAME:
                continue
            if fid != self.last_id:
                self.last_id = fid
                self.frames[fid] = (ccnt, {})
            slot = self.frames.get(fid)
            if slot is None:
                continue
            slot[1][cidx] = data[20:20 + plen]
            # ★★ 2026-10-12：仿真在**第 0 片**的 JPEG 之后追加了 24 字节相机位姿 ★★
            #   正是"拍这张图的相机"。它和图像在同一个 UDP 包里 ⇒ 不存在"图像配新姿态"
            #   的时间差（原来 41001/41002 两条队列会错开几十毫秒，云台一转整帧偏几十度）。
            #   老版本仿真不发这 24 字节 ⇒ 长度不够时保持 None，回落到 41002 的状态包 ✓
            if cidx == 0 and len(data) >= 20 + plen + 24:
                try:
                    n = min(9, (len(data) - (20 + plen)) // 4)
                    _vals = struct.unpack_from("<%df" % n, data, 20 + plen)
                    # 补齐到 9 个（老仿真只发 6 个：位姿，没有相机->枪口偏移）
                    _pose = tuple(_vals) + (0.0,) * (9 - n)
                    # ★★ 关键修复：位姿按**帧号**存，等这一帧拼完时再拿出来。
                    #   旧写法直接覆盖 self.frame_pose —— 同一个 pump() 里积了多帧时，
                    #   "第 N 帧拼完的图"会配上"第 N+1 帧第 0 片带来的位姿" ⇒
                    #   云台一转就差十几度（线上错 30°、离线单帧测试却 0.01°）✓
                    self.frame_pose_map[fid] = _pose
                except struct.error:
                    pass
            if len(slot[1]) == slot[0]:
                got = b"".join(slot[1][k] for k in sorted(slot[1]))
                del self.frames[fid]
                self.last_frame_id = fid      # HUD jitter fix: frame id of the decoded frame
                _p = self.frame_pose_map.pop(fid, None)
                if _p is not None:
                    self.frame_pose = _p      # 与刚返回的这张图**严格同帧** ✓
                    self.frame_pose_id = fid
                if len(self.frames) > 8:
                    for k in sorted(self.frames)[:-4]:
                        del self.frames[k]
                        self.frame_pose_map.pop(k, None)
        while True:
            try:
                d, _ = self.st.recvfrom(256)
            except (BlockingIOError, OSError):
                break
            if len(d) >= 20:
                m, gy, gp, rl, ts = struct.unpack_from("<Iffff", d, 0)
                if m != MAGIC_STATE:
                    continue
                # ★ 2026-10-12：offset 20 是**到目标的距离(cm)**，以前被完全跳过 ✗
                #   （仿真 WLVehiclePawn.cpp:7702 算好 RangeCm 放在这里，注明"给视觉端做弹道补偿用"）
                #   现在收下，作为弹道补偿的备用距离（主用 PnP 解出的斜距）
                if len(d) >= 24:
                    self.range_cm = float(struct.unpack_from("<f", d, 20)[0])
                # 18 个 float 的位姿从 **offset 24** 开始（原来是 20，把距离那 4 字节当成了
                # chassis_x ✗ —— 只差 4 字节、影响极小，但一并纠正 ✓）
                if len(d) >= 24 + 72:
                    ext = struct.unpack_from("<18f", d, 24)
                    self.last_state = ext
                    self.state_raw = (gy, gp, rl, ts,
                                      ext[0:3], ext[3:6],      # chassis pos / rpy
                                      ext[6:9], ext[9:12],     # gimbal  pos / rpy
                                      ext[12:15], ext[15:18])  # muzzle  pos / camera rpy
                else:
                    self.last_state = None
                    self.state_raw = (gy, gp, rl, ts, (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0))
        if got is not None:
            self.n_frames += 1
        return got

    def build_frame(self, dets, t):
        """dets: [(points4x2, color, conf)] -> DetectionFrame"""
        from core.bus.dataflow import Detection, DetectionFrame
        from core.entities.property.entity_types import EntityType
        out = []
        for pts, color, cf in dets:
            out.append(Detection(entity_type=EntityType.Infantry,
                                 image_points_2d=np.asarray(pts, float).reshape(4, 2),
                                 armor_size="small"))
        return DetectionFrame(detections=out, timestamp=t)

    def send_dets(self, dets):
        """把本帧检测到的四角点回传（magic 'RMDT' + uint16 count + float2 x4 x count）。"""
        if not dets:
            pkt = struct.pack("<IH", 0x54444D52, 0)
            self.dtx.sendto(pkt, self.ddst)
            return
        pts = []
        for p, c, cf in dets[:16]:
            arr = np.asarray(p, float).reshape(4, 2)
            for q in arr:
                pts.extend([float(q[0]), float(q[1])])
        pkt = struct.pack("<IH", 0x54444D52, len(dets[:16])) + struct.pack("<%df" % len(pts), *pts)
        self.dtx.sendto(pkt, self.ddst)

    def send_cmd(self, dyaw, dpitch, fire, has_target, t, abs_yaw=None, abs_pitch=None,
                 dets_for_echo=None, echo_frame_id=0):
        """发指令。给了 abs_yaw/abs_pitch 就走**绝对角通道**（位置控制）——
        这正是 PlugYourRM 的 ShotSolution 语义（绝对云台指向），让他们的闭环独占控制，
        不要差分成增量（差分等于对误差求导：噪声放大、越远越晃，且重复 tick 时增量为 0 → yaw 不动 ✗）。"""
        if abs_yaw is not None and abs_pitch is not None:
            # 格式必须与 WLAimNet.cpp 的读取偏移严格一致：
            #   0 magic | 4 dyaw | 8 dpitch | 12 fire | 13 has_target | 14-15 pad | 16 time
            #   20 abs_yaw(float) | 24 abs_pitch(float) | 28 useAbsolute(uint8)  => 共 29 字节
            # 之前写成 "<IffBB2xfB3xff"（flag 在 20、两个 float 在 24/28）✗ ⇒ 仿真读到的
            # absYaw 是 flag+pad+半个 yaw、absPitch 是 yaw 的浮点 ⇒ 现象就是
            # "pitch 跟着横向量走、yaw 不动"（用户当场指出"你把 pitch 和 yaw 搞反了"✓）
            # 检测结果**搭车**在指令包尾部（偏移 29）：uint8 nDets + nDets*4 个 int16 点。
            # 不再走 41004：那条独立通道被"每次 Play 泄漏一个 socket"搞死过 ✗（旧 socket 抢收 ✓）。
            tail = b""
            if dets_for_echo:
                ds = dets_for_echo[:16]
                tail = struct.pack("<BI", len(ds), int(echo_frame_id or 0))
                for p, c, cf in ds:
                    arr = np.asarray(p, float).reshape(4, 2)
                    for q in arr:
                        tail += struct.pack("<hh", int(round(q[0])), int(round(q[1])))
            pkt = struct.pack("<IffBB2xfffB", MAGIC_CMD, 0.0, 0.0,
                              1 if fire else 0, 1 if has_target else 0, t,
                              abs_yaw, abs_pitch, 1) + tail
            self.n_abs += 1
        else:
            pkt = struct.pack("<IffBB2xf", MAGIC_CMD, dyaw, dpitch,
                              1 if fire else 0, 1 if has_target else 0, t)
            self.n_inc += 1
        self.tx.sendto(pkt, self.dst)
        self.n_cmds += 1


# ====================================================================== 主程序
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=r"C:\Users\ASUS\Desktop\北洋机甲\neoautoaim1\TJURM-Aiming\config\models\v5n_7c_blk_640_480_v2.onnx")
    ap.add_argument("--tracker", default="multi_target")
    ap.add_argument("--pose-solver", default="tjurm")
    ap.add_argument("--enemy-color", default=None, help="red/blue；给定就只保留该色")
    ap.add_argument("--no-lightbar", action="store_true", help="只用 YOLO 框四角，不做灯条精修")
    ap.add_argument("--no-box-fallback", action="store_true", help="关闭框四角兜底（默认开：只对已过滤的板状框生效）")
    # 符号用 --synthetic 扫描实测定的（2026-10-11）：
    #   横向 u 300->1150 时他们的 yaw +0.316->-0.491（单调减）⇒ 目标右移 yaw 减小；
    #   我们协议 +dYaw = 炮口右（鼠标已标定）⇒ yaw 要取 -1。写反了在闭环里就是正反馈 → 左右乱晃 ✗
    #   纵向 v 200->950 时他们的 pitch +0.353->-0.511；双方都是"抬头为正" ⇒ pitch 取 +1 ✓
    ap.add_argument("--yaw-sign", type=float, default=1.0, help="绝对角通道下取 +1：他们逆时针为正、仿真 MIRRORED，两次取负抵消（详见 WLVehiclePawn.cpp 的推导）")
    ap.add_argument("--pitch-sign", type=float, default=1.0, help="实测 +1")
    # ===== 2026-10-12：控制通道选择 =====
    #   pixel（默认）= 像素伺服（增量通道）—— 只用检测框中心，不依赖相机姿态、不依赖
    #                  图像与状态包的时间对齐 ⇒ 不会出现"永远朝一个方向转到底" ✓
    #   abs          = 绝对角通道 —— 用 PlugYourRM 的 ShotSolution/世界系目标角，
    #                  理论上有提前量，但**要求图像时刻的相机姿态准确**；实测现场
    #                  两者不一致时会整帧偏几十度 ⇒ 单向甩飞 ✓
    ap.add_argument("--mode", choices=("pixel", "abs"), default="abs",
                    help="控制通道：abs=TF树+实拍PnP的绝对角（默认）、pixel=像素伺服（备用兜底）")
    ap.add_argument("--pixel-gain", type=float, default=0.06,
                    help="像素伺服增益（度/帧 per 100px）。换算：dYaw = gain*px_err/12.5，"
                         "500px 误差 -> 2.4 度/帧（约 70 度/秒 @30fps）。要更快就调大")
    ap.add_argument("--pixel-max-step", type=float, default=6.0,
                    help="像素伺服每帧最大角度增量（度），防止畸形检测把云台一脚踢飞")
    # ★ 2026-10-12：绝对角的现场微调（单位：度，作用在**绝对角**上）
    #   判据来自用户实测："整体偏右、pitch 偏上" ⇒
    #     偏右 -> yaw 要往左补 -> --yaw-trim 取**正**
    #     偏上 -> pitch 要往低压    -> --pitch-trim 取**负**
    #   先按 0 跑一次，看弹着点偏多少再填；每度大约对应 21.8 px（f=1250）。
    ap.add_argument("--yaw-trim", type=float, default=0.0,
                    help="绝对角 yaw 微调（度）。弹着点偏右就调正（往左补）")
    ap.add_argument("--pitch-trim", type=float, default=0.0,
                    help="绝对角 pitch 微调（度）。弹着点偏上就调负（往低压）")
    # ★ 2026-10-12：弹道补偿（远距离必须开，否则打低）。默认按**仿真**参数：17mm/3.2g/23m/s/二次阻力
    ap.add_argument("--no-ballistic", action="store_true",
                    help="关掉弹道补偿（默认开）。只用于对照实验")
    ap.add_argument("--v0", type=float, default=23.0,
                    help="弹丸初速 m/s（仿真 MuzzleSpeedCmS=2300 ⇒ 23.0）")
    ap.add_argument("--cd", type=float, default=0.47, help="弹丸阻力系数（球体 0.47）")
    ap.add_argument("--ballistic-scale", type=float, default=1.0,
                    help="弹道抬枪角的整体缩放（1.0=按物理算；偏大偏小用它微调）")
    # ★ 2026-10-12：稳态误差积分（**默认开** —— 实测它就是"从不同方向跟过来停在不同的角度"的根因）
    #   纯比例控制 + 摩擦会留下与接近方向有关的稳态误差；开了它，两次落点才一致 ✓
    ap.add_argument("--det-consistency", action="store_true", default=False,
                    help="检测一致性预过滤（**默认关**）。开着的两个实测副作用："
                         "① 云台快速平移/换角度看时框本来就会大位移，会被误判成异常挡掉；"
                         "② 目标持续逼近时判据会越挡越死（旧版）。"
                         "默认关掉 = 检测原样进 tracker（与老代码一致），"
                         "tracker 自带的 min_detect_count / 卡方门限会自己剔离群观测")
    ap.add_argument("--no-det-consistency", dest="det_consistency", action="store_false",
                    help="关掉一致性检查（完全照搬老代码 aim_peer.py 的无状态做法）")
    # ★★ 2026-10-12：稳态误差积分 —— **默认关**（曾默认开，是错的）★★
    #   实测教训：目标角本身还有常数残差时，积分器会**一直累加**（0.02/s × 残差 4°
    #   = 0.08°/s）⇒ 表现就是"yaw 缓慢单向飘走" ✓
    #   实测印证：47 秒里 vision yaw 从 -3.21 飘到 -4.92（1.71°），与计算完全吻合；
    #   而同期 TRUE_abs=-3.53、cur_head=+1.85 **都没动** ⇒ 飘的是 peer 发出去的数 ✓
    #   顺序：先把基础角度对准，再用它消"从不同方向跟过来停在不同角度"的稳态误差。
    ap.add_argument("--integ", action="store_true",
                    help="开启稳态误差积分（**默认关**）。基础角度没对准时开它会变成漂移源")
    # ★ 2026-10-12 外参平移（相机在云台系下的安装位置）的增益。
    #   1.0 = 按库的官方链路把平移完整算进去；0.0 = 不计入（默认）。
    #   实测：0.05 m 平移在 2.15 m 上只值 1.33°，而把它加进去会让 yaw 差从 0.04° 变 0.61°
    #   ⇒ 说明相机位置与 PnP 原点口径还没完全对齐。先默认 0，等量到真实安装 xyz 再定。
    ap.add_argument("--extr-t-gain", type=float, default=1.0,
                    help="外参平移计入比例（1=按你的链路完整计入，默认 1）")
    ap.add_argument("--integ-limit", type=float, default=1.0, help="积分项限幅（度），防跑飞")
    ap.add_argument("--integ-rate", type=float, default=0.02, help="积分速率（度/秒 per 度误差）")
    ap.add_argument("--integ-pxgate", type=float, default=60.0,
                    help="只在像素误差小于这个值时积分（确保咬住目标）")
    # ★ 2026-10-12：所有"补偿"的总开关。默认 **0 = 全部关掉**，先量原始偏差再逐个打开。
    #   1 = 启用"枪口→目标"瞄准线（相机->枪口偏移补偿）
    #   2 = 同时启用 --yaw-trim/--pitch-trim
    ap.add_argument("--comp", type=int, default=0, choices=(0, 1, 2),
                    help="补偿级别：0=全关(默认,量原始值) 1=只开枪口瞄准线 2=连 trim 一起开")
    ap.add_argument("--bias-frames", type=int, default=90,
                    help="自动配平采样帧数（敲 b 后采这么多帧的平均值，90 帧≈1.5s）")
    ap.add_argument("--verbose", action="store_true",
                    help="打印 [tf] RAW / [abs] 详细诊断（默认关：只留底部一行实时状态，不刷屏）")
    ap.add_argument("--dry", default=None, help="离线自测：从这个目录的图片跑，不发 UDP")
    ap.add_argument("--synthetic", action="store_true",
                    help="离线合成扫描：直接喂四角点，测他们 yaw 的符号约定")
    ap.add_argument("--limit", type=int, default=40)
    args = ap.parse_args()

    # ★★ 启动横幅：一眼看清"跑的是哪一版"（用户此前要靠问才知道，太费事）★★
    import os as _os
    _mt = _os.path.getmtime(__file__)
    import datetime as _dt
    print("=" * 78)
    print("[peer] 代码版本 : %s   文件改动时间 : %s"
          % (_os.path.basename(__file__),
             _dt.datetime.fromtimestamp(_mt).strftime("%Y-%m-%d %H:%M:%S")))
    print("[peer] 链路     : 板系3D + 2D角点 + 内参 K → PnP(相机外参 M=R_chassis^T·R_camera)"
          " → 固定云台系 → 世界")
    print("[peer] 关键约定 : pitch 输入取负(库 Ry 是 nose-down 正) | 发绝对语义(code=1)"
          " | 偏置=常数残差 | 积分默认关 | 预过滤默认关")
    print("[peer] 弹道     : v0=%.1f m/s  Cd=%.2f  开=%s  距离源=仿真状态包"
          % (args.v0, args.cd, "是" if not args.no_ballistic else "否"))
    print("=" * 78)

    # 把弹道参数灌进 BALLISTIC（供 ballistic_superelevation 用）
    BALLISTIC['enable'] = not args.no_ballistic
    BALLISTIC['v0'] = float(args.v0)
    BALLISTIC['cd'] = float(args.cd)
    BALLISTIC['cache'].clear()
    INTEG['lim'] = float(args.integ_limit)
    INTEG['rate'] = float(args.integ_rate)
    INTEG['pxgate'] = float(args.integ_pxgate)

    # ---- 装总线 + boot 他们的 custom（完全按 entrance/session.py 的方式）----
    from core.bus import EventBus, install_sys_bus, install_public_bus
    install_sys_bus(EventBus())
    install_public_bus(EventBus())
    from custom.comm import CustomComm
    from core.bus import public_bus
    from core.bus.messages import PublicMessages
    custom_comm = CustomComm()
    import custom
    pipeline = custom.boot(comm=custom_comm, tracker_name=args.tracker, pose_solver_name=args.pose_solver)
    print("PlugYourRM boot: tracker=%s pose_solver=%s" % (args.tracker, args.pose_solver))

    # ---- 他们的输出：ShotSolution（绝对角度）----
    solved = {"n": 0, "last": None}

    def on_control(sol):
        solved["n"] += 1
        solved["last"] = sol
    public_bus().subscribe(PublicMessages.CONTROL, on_control)

    vis = VisionFront(args.model, use_lightbar=not args.no_lightbar,
                      enemy_color=args.enemy_color, box_fallback=not args.no_box_fallback)
    ssb = SelfStateBuilder()

    if args.dry:
        # ---------------- 离线自测：只验"我们的检测能不能喂出他们的 ShotSolution" ----------------
        import glob
        files = sorted(glob.glob(os.path.join(args.dry, "*.png")) + glob.glob(os.path.join(args.dry, "*.jpg")))[:args.limit]
        n_ok = 0
        corr = []
        for i, f in enumerate(files):
            raw = np.fromfile(f, dtype=np.uint8)
            dets, nbox, nref = vis.detect(raw.tobytes())
            if dets is None:
                continue
            t = i / 30.0
            # 离线时自车状态未知：先喂一个"相机在原点、朝 +x、水平"的占位状态，
            # 只为验证 pose_solver/tracker 能否跑通（距离/角度无意义）。
            ssb.update((0.0, 0.0, 0.0, t, (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)))
            public_bus().publish(PublicMessages.DETECTION, _frame(dets, t))
            public_bus().publish(PublicMessages.SELF_STATE, ssb.build(t))
            pipeline.tick()
            if solved["n"] > n_ok:
                n_ok = solved["n"]
                s = solved["last"]
                try:
                    _d = solved.get("dets") or []
                    _info = " ".join("c=(%.0f,%.0f)w=%.0f" % (
                        float(np.asarray(p, float)[:, 0].mean()), float(np.asarray(p, float)[:, 1].mean()),
                        float(np.asarray(p, float)[:, 0].max() - np.asarray(p, float)[:, 0].min()))
                        for p, c, cf in _d[:3])
                    _corners = ""
                    if _d:
                        _c0 = np.asarray(_d[0][0], float)
                        _corners = "pts=" + " ".join("(%.0f,%.0f)" % (a, b) for a, b in _c0)
                    print("[dbg] n_dets=%d %s | %s | sol yaw=%s pitch=%s fire=%s" % (
                        len(_d), _info, _corners,
                        ("%.1f" % math.degrees(s.yaw)) if s else "None",
                        ("%.1f" % math.degrees(s.pitch)) if s else "None",
                        s.fire if s else "-"))
                except Exception as _e:
                    print("[dbg] err", _e)
                # 打印"装甲板在画面里的横向位置 u"与"他们的绝对 yaw" —— 用相关性判定符号：
                # 目标在画面里越靠右(u 大)，云台要往哪边转，就能推出我们协议该用 +1 还是 -1。
                u = float(np.mean([p[0] for p, c, cf in dets]))
                v = float(np.mean([p[1] for p, c, cf in dets]))
                print("  [%03d] %s boxes=%d refined=%d u=%.0f v=%.0f -> yaw=%.3f pitch=%.3f fire=%s"
                      % (i, os.path.basename(f), nbox, nref, u, v, s.yaw, s.pitch, s.fire))
                corr.append((u, v, s.yaw, s.pitch))
        print("dry done: %d frames, %d ShotSolution" % (len(files), solved["n"]))
        if len(corr) >= 2:
            import numpy as _np
            A = _np.array(corr, float)
            cu = _np.corrcoef(A[:, 0], A[:, 2])[0, 1]      # u 与 yaw 的相关
            cv = _np.corrcoef(A[:, 1], A[:, 3])[0, 1]      # v 与 pitch 的相关
            print("sign probe: corr(u,yaw)=%+.2f  corr(v,pitch)=%+.2f" % (cu, cv))
        return

    if args.synthetic:
        # ---------------- 合成扫描：直接喂四角点，测"目标横向位置 -> 他们的 yaw"的符号 ----------------
        # 固定自车状态（相机在原点、朝 +x、水平），只让目标的 u（横向像素）变化。
        # 于是 sol.yaw 随后 u 单调变化的方向，就是他们 yaw 的符号约定。
        print("synthetic sweep: u -> their ShotSolution (fixed self-state)")
        rows = []
        for u in (300, 450, 600, 750, 900, 1050, 1150):
            yaw = pitch = None
            for k in range(12):                      # 每个位置喂 12 帧，让他们的 tracker 收敛
                t = (len(rows) * 12 + k) / 30.0
                w, h = 120.0, 60.0
                pts = np.array([[u - w / 2, 500 - h / 2], [u + w / 2, 500 - h / 2],
                                [u + w / 2, 500 + h / 2], [u - w / 2, 500 + h / 2]], float)
                dets = [(pts, "red", 0.9)]
                ssb.update((0.0, 0.0, 0.0, t, (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)))
                public_bus().publish(PublicMessages.DETECTION, _frame(dets, t))
                public_bus().publish(PublicMessages.SELF_STATE, ssb.build(t))
                pipeline.tick()
                if solved["last"] is not None:
                    yaw = solved["last"].yaw
                    pitch = solved["last"].pitch
            rows.append((u, yaw, pitch))
            print("  u=%4d -> yaw=%s pitch=%s" % (u,
                  ("%.4f" % yaw) if yaw is not None else "None",
                  ("%.4f" % pitch) if pitch is not None else "None"))
        ys = [(u, y) for u, y, p in rows if y is not None]
        if len(ys) >= 2:
            du = ys[-1][0] - ys[0][0]
            dy = ys[-1][1] - ys[0][1]
            print("=> d(u)=%+.0f  d(yaw)=%+.4f  =>  yaw %s when the target moves RIGHT"
                  % (du, dy, "INCREASES" if dy > 0 else "DECREASES"))
            print("   our protocol: +dYaw = turret RIGHT (calibrated with the mouse)")
            print("   => use --yaw-sign %s" % ("+1" if dy > 0 else "-1"))

        # 纵向扫描：v 变大 = 目标在画面里更低。他们的 pitch 契约是"抬头为正"，
        # 我们的 +dPitch = 炮口上抬。两者符号是否一致，用这个扫描判定。
        print("synthetic sweep (vertical): v -> their pitch")
        rows2 = []
        for v in (200, 350, 500, 650, 800, 950):
            pit = None
            for k in range(12):
                t = 10.0 + (len(rows2) * 12 + k) / 30.0
                w, h = 120.0, 60.0
                pts = np.array([[640 - w / 2, v - h / 2], [640 + w / 2, v - h / 2],
                                [640 + w / 2, v + h / 2], [640 - w / 2, v + h / 2]], float)
                ssb.update((0.0, 0.0, 0.0, t, (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)))
                public_bus().publish(PublicMessages.DETECTION, _frame([(pts, "red", 0.9)], t))
                public_bus().publish(PublicMessages.SELF_STATE, ssb.build(t))
                pipeline.tick()
                if solved["last"] is not None:
                    pit = solved["last"].pitch
            rows2.append((v, pit))
            print("  v=%4d -> pitch=%s" % (v, ("%.4f" % pit) if pit is not None else "None"))
        vs = [(v, p) for v, p in rows2 if p is not None]
        if len(vs) >= 2:
            dv = vs[-1][0] - vs[0][0]
            dp = vs[-1][1] - vs[0][1]
            print("=> d(v)=%+.0f  d(pitch)=%+.4f  =>  pitch %s when the target moves DOWN"
                  % (dv, dp, "INCREASES" if dp > 0 else "DECREASES"))
            print("   their contract: pitch>0 = barrel UP (抬头为正); our +dPitch = barrel UP")
            print("   => use --pitch-sign %s" % ("+1" if dp < 0 else "-1"))
        return
        return

    # ---------------- 在线：UDP + keeper 线程 ----------------
    bridge = UdpBridge()
    # 启动时自动载入上次存的偏置（如果按过 y）
    try:
        import json as _json
        _bp = os.path.join(TOOLS, "bias_saved.json")
        if os.path.exists(_bp):
            with open(_bp, encoding="utf-8") as _fh:
                _b = _json.load(_fh)
            AUTOBIAS['yaw'] = float(_b.get('yaw', 0.0))
            AUTOBIAS['pitch'] = float(_b.get('pitch', 0.0))
            AUTOBIAS['active'] = True
            print("[bias] loaded saved bias: yaw=%+.2f pitch=%+.2f" % (AUTOBIAS['yaw'], AUTOBIAS['pitch']))
    except Exception as _e:
        print("[bias] load skipped: %s" % _e)

    keeper = threading.Thread(target=_keeper, args=(pipeline, bridge, solved, args, ssb), daemon=True)
    keeper.start()
    _start_bias_console()
    _biashdr = ("bias window:  <- -> yaw0.1   ^ v pitch0.1   PgUp/PgDn pitch1.0   "
                "b=auto   r=clear   y=save   ?=help")
    print("waiting for the sim's frames on udp/41001 (start it with -WLAimNet, or press O in game)...")
    print(_biashdr)
    print("[bias] current yaw=%+.2f pitch=%+.2f" % (AUTOBIAS['yaw'], AUTOBIAS['pitch']))
    last_log = time.time()
    last_sol = None
    stat = {'last': 'n/a'}
    while True:
        jpg = bridge.pump()
        now = time.time()
        if now - last_log > 1.0:
            last_log = now
            if bridge.n_frames == 0:
                _live("WAITING: 0 frames - is the sim running? (type WLAimNet 1 / press O in game)")
            else:
                s = solved["last"]
                _live("[tuner] bias y=%+.2f p=%+.2f | frames=%d sol=%d | %s | %s"
                      % (AUTOBIAS['yaw'], AUTOBIAS['pitch'], bridge.n_frames,
                         solved["n"],
                         ("yaw=%.2f pitch=%.2f fire=%s" % (s.yaw, s.pitch, s.fire)) if s else "no solution yet",
                         stat.get("last")))
        if jpg is None:
            time.sleep(0.001)
            continue
        dets, nbox, nref = vis.detect(jpg)
        # ★★ 2026-10-12：把"这张图的帧号 + 拍它的相机位姿"取成快照 ★★
        #   检测本身要跑几十毫秒，期间 pump() 会一直更新 bridge.frame_pose ⇒ 直接读就会
        #   拿到**更新的**帧的姿态，等于又把时间差引回来 ✗。所以在这里立刻快照，
        #   后面 PnP 只用这份快照（与图像严格同一帧 ✓）。
        if dets is not None and len(dets) > 0:
            # ★★ 2026-10-12：检测一致性检查（替代原来那道错的闸门）★★
            #   为什么原来那道是错的（用户指出）：用**固定 25% 像素变化**当判据 ⇒
            #   完全没有"进大远小"的概念。目标正常靠近/远离时框本来就在持续缩放 ⇒ 必然误判 ⇒
            #   而且被丢弃的帧不更新基准 ⇒ 永久卡死 ⇒ "太近太远都没有检测" ✗
            #
            #   为什么老代码 aim_peer.py 打弹不受影响：它**完全无状态** ——
            #       ex = 框心 - 画面心;  raw_yaw = gain*ex/(W/2)*10;  max_step 钳位
            #   没有历史、没有中位数、没有闸门。弹丸糊一帧 ⇒ 只错那一帧 ⇒ 下一帧自动恢复 ✓
            #   而现在这条链有 EKF + tracker 的 ID 匹配 + PnP，**有状态** ⇒ 一帧垃圾污染好几帧。
            #
            #   所以判据改成"按框大小归一"（= 按距离归一），并且只挡**单帧尖峰**：
            #       正常帧：框心/框尺寸逐帧连续变化        -> 通过
            #       弹丸帧：框心或框尺寸出现**一次**尖峰     -> 本帧不喂给 tracker，沿用上一帧观测
            #   阈值是"框尺寸的倍数"而不是固定像素：|Δcenter| < 1.5*box_w（近距离大框允许大位移）
            _c = np.asarray(dets[0][0], float).reshape(4, 2)
            _ctr = (_c[:, 0].mean(), _c[:, 1].mean())
            _w = float(_c[:, 0].max() - _c[:, 0].min())
            _h = float(_c[:, 1].max() - _c[:, 1].min())
            _scale = max(_w, _h, 8.0)          # 用框自身尺度做归一 ⇒ 天然"进大远小"
            _prev = solved.get('_det_prev2')
            _bad = None
            if args.det_consistency and _prev is not None:
                _dpix = ((_ctr[0] - _prev[0]) ** 2 + (_ctr[1] - _prev[1]) ** 2) ** 0.5
                _dsz = abs(_w - _prev[2]) / max(_prev[2], 1.0)
                # 位移门限 = 框宽的 1.5 倍（正常运动远小于此；弹丸/误检是整块跳走）
                if _dpix > 1.5 * max(_prev[2], 8.0):
                    _bad = "jump %.1f*box" % (_dpix / max(_prev[2], 1.0))
                # 尺寸门限 60%：正常缩放（进大远小）是连续的，不会单帧跳 60%
                elif _dsz > 0.60:
                    _bad = "size %+.0f%%" % (_dsz * 100.0)
            if _bad is not None:
                solved['_det_rej'] = solved.get('_det_rej', 0) + 1
                # ★ 关键：丢弃时**不动** _det_prev2，下一帧仍与"最后一个良好帧"比 ⇒
                #   弹丸只影响它自己那一帧，不会像旧闸门那样把基准锁死 ✓
                if solved['_det_rej'] % 5 == 1:
                    _live_done()
                    print("[det] hold frame (%s) w=%.0f h=%.0f | total %d"
                          % (_bad, _w, _h, solved['_det_rej']))
                if dets is not None:
                    solved['frame_id'] = bridge.last_frame_id
                    solved['frame_pose'] = bridge.frame_pose
                _live("[tuner] bias y=%+.2f p=%+.2f | 单帧异常已挡 (%s) 累计%d"
                      % (AUTOBIAS['yaw'], AUTOBIAS['pitch'], _bad, solved['_det_rej']))
                time.sleep(0.001)
                continue
            solved['_det_prev2'] = (_ctr[0], _ctr[1], _w)
        if dets is not None:
            solved['frame_id'] = bridge.last_frame_id
            solved['frame_pose'] = bridge.frame_pose
        if dets is None:
            continue
        cols = {}
        for p, c, cf in dets:
            cols[str(c)] = cols.get(str(c), 0) + 1
        SHARED['px_err'] = float(np.asarray(dets[0][0], float)[:, 0].mean() - CAM_W / 2.0) if dets else None
        SHARED['px_err_v'] = float(np.asarray(dets[0][0], float)[:, 1].mean() - CAM_H / 2.0) if dets else None
        stat['last'] = "plates=%d refined=%d classes=%s colors=%s" % (nbox, nref, getattr(vis, 'last_classes', []), cols)
        solved['dets'] = dets       # 存起来，让 keeper 搭在指令包尾部回传给仿真 ✓
        solved['last_fid'] = bridge.last_frame_id   # 这批检测来自哪一帧（HUD 靠它对帧 ✓）
        # ★ 2026-10-13: 10 -> 300（180fps 下每 10 帧写一次 25KB 会拖慢主循环）
        if bridge.n_frames % 10 == 0:
            try:
                os.makedirs(r"D:\DeepseekPlungin\sim_frames", exist_ok=True)
                with open(r"D:\DeepseekPlungin\sim_frames\f_%05d.jpg" % bridge.n_frames, "wb") as fh:
                    fh.write(jpg)
            except Exception:
                pass
        t = now
        bridge.state_raw and ssb.update(bridge.state_raw)
        public_bus().publish(PublicMessages.DETECTION, _frame(dets, t))
        st = ssb.build(t)
        if st is not None:
            public_bus().publish(PublicMessages.SELF_STATE, st)


def _frame(dets, t):
    from core.bus.dataflow import Detection, DetectionFrame
    from core.entities.property.entity_types import EntityType
    # ★ 像素必须**以图像中心为原点**（他们文档 §5.1：主点 (0,0)，K 也是主点 0 的形式）。
    # 我们算出来的角点是左上角坐标系，这里减掉主点；否则 PnP 会把每个目标都偏 (640,512) px ✗
    C = np.array([CAM_W / 2.0, CAM_H / 2.0], float)
    # ★ 防御：畸形检测（形状不是 8 个数 / 不是 (4,2)）直接丢弃 ✓，绝不让整个进程崩 ✗
    #   （实测：某条路径会给出 size=4 的点 ⇒ reshape(4,2) 抛 ValueError ⇒ 进程退出 ✗）
    out, bad = [], 0
    for p_, c_, cf_ in dets:
        a = np.asarray(p_, float).reshape(-1)
        if a.size == 8:
            a = a.reshape(4, 2)
        else:
            bad += 1
            continue
        out.append(Detection(entity_type=EntityType.Infantry,
                             image_points_2d=(a - C), armor_size="small"))
    if bad:
        global _BAD_WARNED
        try:
            _BAD_WARNED
        except NameError:
            _BAD_WARNED = False
        if not _BAD_WARNED:
            _BAD_WARNED = True
            print("[warn] dropped %d malformed detection(s) - shapes were not 8 floats ✓" % bad)
    return DetectionFrame(detections=out, timestamp=t)


def _keeper(pipeline, bridge, solved, args, ssb):
    """按 60 Hz 叫 pipeline.tick()，并把 ShotSolution 的绝对角差分成我们协议的增量。

    为什么差分：ShotSolution 是**绝对**云台角，我们的协议是**每帧增量**（视觉伺服）。
    """
    from core.bus import public_bus
    from core.bus.messages import PublicMessages
    prev = None
    last_sent_n = -1
    use_direct_sent = False
    last_hb_frame = -1
    while True:
        try:
            pipeline.tick()
            sol = solved["last"]
            # ★ 只有**新的**解才发（2026-10-11 实测到的 bug）：原来看 `sol is not None` 就发，
            # 于是仿真停了、没有新帧时，keeper 仍以 58 Hz 重发同一条陈旧指令
            # （日志里 frames 冻在 3559、cmds 从 1321 一路涨到 30069 ✗✗）⇒ 云台被陈旧指令一直顶住，
            # 现场表现就是"一进自瞄 pitch 就压在限位上不动"。改为：解的序号没变就不发。
            # 每帧都发一个"检测心跳"：有解时带绝对角 + 检测，没解时只带检测（角度全 0、useAbsolute=0，
            # 对云台无影响），这样[目标出视野]能立刻传到仿真，绿框立刻消失 ✓（用户要求）。
            if (sol is None) or (solved["n"] == last_sent_n):
                if bridge.n_frames != last_hb_frame:
                    last_hb_frame = bridge.n_frames
                    bridge.send_cmd(0.0, 0.0, False, bool(solved.get("dets")), time.time(),
                                    dets_for_echo=solved.get("dets"),
                                    echo_frame_id=solved.get("last_fid", 0))
            if sol is not None and solved["n"] != last_sent_n:
                last_sent_n = solved["n"]
                # ★ 用移植的 YawPnP 覆盖 yaw（用户那版老代码的做法 ✓：yaw 当一维未知量三分搜索 ⇒ 无二义性 ⇒ 不甩 ✓）
                # ? ?????**??**?armor_solver(PnP) + abs_camera_to_gimbal ? G ??????
                #   ??**??**? target yaw/pitch ? ?? ????"G ??? ? ????? yaw/pitch ??" ??
                #   ???? tracker/MPC ???????????????? ? ?? CSV ???? +1.03 ??
                # ===== 相机系求方位（2026-10-12 重写；旧版 Rg 分解已废弃）=====
                # 步骤：
                #   1) PnP（armor_solver）用 rot_camera_to_abs_camera 解出 pos_c —— 目标在 C'
                #   2) 用相机的三个世界轴把 pos_c 投影成（前/右/上）三个分量
                #   3) yaw_rel  = atan2(右, 前)    目标在画面右为负、在左为正
                #      pitch_rel= atan2(上, hypot(前,右))
                #   4) 绝对角 = 相机世界朝向 + 相对角（同一个"正=左"约定）
                # 不经过云台位置、不构造 Rg ⇒ 云台基准点、左右手性、姿态重复计入
                # 这三类错一并消失（旧版三样全中，见下面 ① ②）。
                use_direct = None
                try:
                    _d = (solved.get("dets") or [None])[0]
                    _st = bridge.last_state
                    if _d is not None and _st is not None:
                        from core.algorithms.math.transform import rpy_to_R
                        from custom.components.armor_solvers import get_solver
                        from custom.components.armor_solvers import ippe as _ippe
                        if G_SOLVER[0] is None:
                            # ★ 2026-10-12：改用 ippe。实测（完美合成输入、深度精确 6.000 m）：
                            #   ippe 6.000 m（0% 误差）、qcqp 8.388 m（+39.8%）、diff 8.388 m（+39.8%）
                            #   qcqp/diff 只约束方向（对整体缩放不变）⇒ 深度尺度没有可观测量 ⇒
                            #   系统性偏大，方位偏 30° 时涨到 +56.8%。
                            #   桥接层只用方向，但距离会进弹道解算 ⇒ 必须用能还原真实尺度的 ippe。
                            G_SOLVER[0] = get_solver("ippe")
                        pr = np.asarray(_d[0], float).reshape(4, 2) - np.array([CAM_W / 2.0, CAM_H / 2.0])
                        # ★★ 2026-10-12：优先用**帧内携带**的相机位姿（与图像同一包、同一时刻）★★
                        #   取的是主循环在检测那一刻做的快照（solved['frame_pose']），
                        #   不是"最新帧"的姿态 —— 检测滞后几帧，直接读最新值等于把时间差引回来 ✗
                        _fp = solved.get("frame_pose")
                        if _fp is None:
                            _fp = bridge.frame_pose        # 兜底：没有快照时用最近一帧
                        if _fp is not None:
                            cr, cp, cy = _fp[5], _fp[4], _fp[3]     # Roll, Pitch, Yaw（度，正=左）
                            cam_pos_ = np.array([_fp[0], _fp[1], _fp[2]], float)
                            _pose_src = "frame#%s" % solved.get("frame_id", "?")
                        else:
                            cy, cp, cr = _st[15], _st[16], _st[17]
                            cam_pos_ = np.array(_st[12:15], float) / 100.0
                            _pose_src = "state"
                        # ★★★★★ 2026-10-12 严格按用户规定的链路重写 ★★★★★
                        # 用户定义的流程：
                        #   1) 装甲板建自己的局部坐标系 → 板系 3D 点（ippe.object_points：LT,RT,RB,LB）
                        #   2) 图像上识别到的 2D 角点
                        #   3) 相机内参 K
                        #   4) 相机外参（**相对固定的云台系**）：
                        #        xyz = 安装位置（实测/由模型量出）
                        #        yaw/pitch/roll = 运行中云台转动的角度（实时）
                        #   5) PnP：板系 3D 点 → 相机系
                        #   6) 相机外参：相机系 → 当前云台系
                        #   7) 目标角 = atan2(y, x)（在云台系里取）
                        #
                        # ★ 实现上的关键事实（读 custom/components/armor_solvers/ippe.py:196-209）：
                        #     t_c1  = B @ t_pnp                 # 板在相机系(FLU)
                        #     M     = rot_camera_to_abs_camera
                        #     t_abs = M @ t_c1                  # ★ 返回值**已经**转到 M 定义的系了
                        #   ⇒ 所以**必须把相机外参当作 M 传进去**，返回值就是"目标在云台系的位置"，
                        #     再取 atan2 就是第 7 步。**绝不能再乘 R/R^T** —— 那等于把外参
                        #     乘两三遍，正是之前 20~41° 误差的来源 ✓
                        #
                        # 相机外参的旋转 = 相机相对固定云台系的旋转。固定云台系固连在底盘上：
                        #     R_cam_in_gimbal = R_chassis^T @ R_camera
                        #   两者都取自状态包（实测值），不假设任何东西。
                        # 安装平移（云台系下的相机光心）：
                        #     t_cam_in_gimbal = R_cam_in_gimbal @ (cam_pos − gimbal_pos)
                        # ★★★ 2026-10-13 定案（撤回上一版的 yaw 取负）★★★
                        #   上一版我据"PIXEL真值=+33.79"把 cy 取了负 —— 那是**我自己探针的公式**
                        #   （-cy+方位）算出来的，不是仿真的 TRUE_abs，属于循环论证 ✗
                        #   按仿真自己的公式推：cam_yaw = atan2(-fwd.Y,fwd.X)，
                        #   TRUE_abs = atan2(-DT.Y,DT.X) ⇒ **TRUE_abs = cam_yaw + 目标相对相机的左偏角**
                        #   ⇒ 应当用 **+cy**，不能取负 ✓
                        #   交叉验证：昨天用仿真真值核过的那一版（cy 不取负）实测
                        #       peer 输出 +1.43  vs  仿真 TRUE_abs +1.41   ⇒ 差 0.06° ✓
                        #   ⇒ 恢复 +cy；pitch 仍取 -cp（库的 Ry 是 nose-down 正方向）
                        _Rch = rpy_to_R(np.deg2rad([_st[3], _st[4], _st[5]]))    # 底盘姿态
                        _Rcam = rpy_to_R(np.deg2rad([cr, -cp, cy]))           # 相机姿态
                        cam_rpy = np.deg2rad([cr, -cp, cy])   # 供后面的调试打印用（同一份约定）
                        _M = _Rch.T @ _Rcam                                    # 相机 -> 固定云台系
                        # ★ 外参 xyz：**两个位置必须来自同一个包**。帧包与状态包的坐标约定不同
                        #   （实测拿帧包位置减状态包云台位置 ⇒ y 分量 −12.1 m，明显不是安装偏移 ✗），
                        #   所以这里统一用状态包：相机位置 _st[12:15]、云台位置 _st[6:9]（都是 cm，仿真世界系）
                        _gp_w = np.array(_st[6:9], float) / 100.0               # 云台世界位置(m)
                        _cam_w = np.array(_st[12:15], float) / 100.0            # 相机世界位置(m)，同包
                        # ★★★ 2026-10-13 关键修复：必须用**相机→炮口**的偏移 ★★★
                        #   仿真真值是按炮口算的：Rel=(BestT-MuzzleWb)/100 (WLVehiclePawn.cpp:7849)
                        #   原来用 cam_w-gp_w = 相机相对**云台转轴**，少了枪管那一段(~0.25m)
                        #   ⇒ 用前进 0.25m 的点去瞄：atan(横/(R-0.25)) > atan(横/R)
                        #   ⇒ 误差**随目标左右翻符号** = 用户报的"左看左偏、右看右偏" ✗
                        #   帧包第 6,7,8 个 float 就是 Dx,Dy,Dz = 相机→炮口 ✓
                        _fp = getattr(bridge, "frame_pose", None)
                        if _fp is not None and len(_fp) >= 9 and \
                                (abs(_fp[6]) + abs(_fp[7]) + abs(_fp[8])) > 1e-6:
                            _t_cam = np.array([float(_fp[6]), float(_fp[7]), float(_fp[8])])
                            _t_src = "帧包D(相机->炮口)OK"
                        else:
                            _t_cam = _M @ (_cam_w - _gp_w)
                            _t_src = "状态包(相机->云台轴)BAD"
                        pos_c, _rpy_c = G_SOLVER[0].estimate(
                            camera_k=K, armor_size="small", image_points_2d=pr,
                            rot_camera_to_abs_camera=_M,
                            mount_pitch=math.radians(ARMOR_TILT_DEG))
                        if pos_c is not None:
                            _pc = np.asarray(pos_c, float)
                            # 第 6 步的后半：把原点从相机光心移到云台原点
                            _pc = _pc + _t_cam * float(getattr(args, "extr_t_gain", 1.0))
                            d_world = _pc
                            # 诊断：把外参 xyz 打出来（每秒一条）
                            if (time.time() - solved.get("_extrt", 0.0)) >= 2.0:
                                solved["_extrt"] = time.time()
                                _live_done()
                                print("[extr] 相机安装 xyz(云台系, m) = (%+.4f, %+.4f, %+.4f) |t|=%.4f"
                                      % (_t_cam[0], _t_cam[1], _t_cam[2],
                                         float(np.linalg.norm(_t_cam))))
                                print("       对照 data.yaml camera_mount_pos = [0.05, 0.00, 0.00]"
                                      "  安装旋转按你的要求 = 运行时云台角（R_chassis^T @ R_camera）")
                            # ★★ 2026-10-12：把瞄准线从"相机→目标"换成"**枪口→目标**" ★★
                            #   相机有安装偏置（实测 cam 比 muzzle 低 ~1 cm、横向偏 ~0.75 cm）⇒
                            #   用相机位姿算出来的角与真实弹道不平行 ⇒ 视觉明明瞄着板心，
                            #   炮口却偏上/偏一侧（用户实测"整体偏右、pitch 偏上"✓）。
                            #   有帧内偏移量时：目标世界位置 = cam_pos + d_world，
                            #   再取 muzzle -> target 的方向 ⇒ 与弹道严格一致 ✓
                            _fp2 = solved.get("frame_pose") or bridge.frame_pose
                            if args.comp >= 1 and _fp2 is not None and len(_fp2) >= 9:
                                _delta = np.array([_fp2[6], _fp2[7], _fp2[8]], float)
                                if float(np.linalg.norm(_delta)) > 1e-6:
                                    _d_muzzle = d_world - _delta
                                    # 只在方向有意义时采用（避免极近距退化）
                                    if float(np.linalg.norm(_d_muzzle)) > 0.05:
                                        d_world = _d_muzzle
                            # ===== 第 7 步：在**云台系**里取角 =====
                            #   _pc = 目标在固定云台系下的位置（第 5、6 步已由 estimate 用外参 M 完成）
                            _yaw_g = math.degrees(math.atan2(_pc[1], _pc[0]))
                            _pitch_g = math.degrees(
                                math.atan2(_pc[2], math.hypot(_pc[0], _pc[1])))
                            # 协议对接：仿真绝对角通道要的是"世界朝向"。
                            #   固定云台系在世界里的姿态就是底盘姿态 R_ch ⇒ 世界向量 = R_ch · 云台系向量
                            #   （代数上 = R_cam · (相机系向量)，与仿真的 TRUE_abs 定义完全同源 ✓）
                            # ===== 第 7 步：目标角 =====
                            # 用户定义的链路：
                            #   板系3D + 2D角点 + 内参K --PnP--> **相机系** --相机外参--> **云台系**
                            #   相机外参：xyz = 实测安装位置；rpy = 运行时云台转角（都相对固定云台系）
                            #   ⇒ 发出去的角 = **云台系下、相对炮口的目标角**（就是一个"要转多少"）
                            #   这样仿真只要把这个角**清零**即可（= 炮口转到目标上），不需要它知道世界系，
                            #   也不需要 peer 去掺相机的世界朝向 cy ✓（与老像素伺服的稳定结构一致）
                            #
                            #   _pc 已经是"目标在固定云台系下的位置"（estimate 内部用 M 转过 ✓）
                            #   相机在该系下的姿态就是 M ⇒ 相机系向量 = M^T @ _pc
                            _t_c1 = _M.T @ _pc
                            # ★★★ 2026-10-13 用户要求：**实测装甲板的真实倾角** ★★★
                            #   解算器 estimate() 把姿态扔了（返回 [0, mount_pitch, psi]），
                            #   所以这里直接调 solve_homography_pnp 自己解一次，取板的"上方向"
                            #   up = R_c1[:,2]（相机系 x前 y左 z上）；与铅垂的夹角 = 板子真实倾角
                            #   板竖直 ⇒ tilt=0；板顶向后仰 ⇒ tilt>0
                            _tilt_deg = None
                            _wh_ratio = None
                            try:
                                _obj = _ippe.object_points("small")
                                _Rp, _tp = _ippe.solve_homography_pnp(pr, _obj, K)
                                if _Rp is not None:
                                    try:
                                        _B = _ippe.RDF2FLU()
                                    except Exception:
                                        _B = np.diag([1.0, -1.0, -1.0])
                                    _Rc = _B @ _Rp
                                    _up = _Rc[:, 2]
                                    _tilt_deg = math.degrees(
                                        math.atan2(-float(_up[0]), float(_up[2])))
                                    # 交叉验证：四边形上/下边宽度比（板倾斜时上边更远 ⇒ 更窄）
                                    _wt = float(np.linalg.norm(np.array(pr[1]) - np.array(pr[0])))
                                    _wb = float(np.linalg.norm(np.array(pr[3]) - np.array(pr[2])))
                                    if _wb > 1e-6:
                                        _wh_ratio = _wt / _wb
                            except Exception as _e:
                                _tilt_deg = None
                            # ★★★ 2026-10-13 用户点破的关键问题：装甲板并非都垂直于地面！★★★
                            #   实测：正面板倾角 0.00°，**侧面板倾角 -15.87°**
                            #   而 peer 原来一律用 ARMOR_TILT_DEG=0 去解 ⇒ PnP 方位偏 9.09° ✗
                            #   修法：**用第一遍解出来的实测倾角，再解一遍**（自标定，任何板都对）✓
                            if _tilt_deg is not None and \
                                    abs(_tilt_deg - ARMOR_TILT_DEG) > 0.5:
                                try:
                                    _pos2, _ = G_SOLVER[0].estimate(
                                        camera_k=K, armor_size="small", image_points_2d=pr,
                                        rot_camera_to_abs_camera=_M,
                                        mount_pitch=math.radians(_tilt_deg))
                                    if _pos2 is not None:
                                        _yaw_before = _gim_yaw
                                        _pc = np.asarray(_pos2, float) + _t_cam * args.extr_t_gain
                                        _t_c1 = _M.T @ _pc
                                        _gim_yaw = math.degrees(math.atan2(_t_c1[1], _t_c1[0]))
                                        _gim_pitch = math.degrees(math.atan2(
                                            _t_c1[2], math.hypot(_t_c1[0], _t_c1[1])))
                                        target_yaw = _gim_yaw
                                        target_pitch = _gim_pitch
                                        if (time.time() - solved.get("_trl", 0.0)) >= 1.0:
                                            solved["_trl"] = time.time()
                                            print("[tilt] ★已用实测倾角 %+.2f° 重解："
                                                  "yaw %+.2f° ← %+.2f°（修正 %+.2f°）"
                                                  % (_tilt_deg, _gim_yaw, _yaw_before,
                                                     _gim_yaw - _yaw_before))
                                except Exception as _e:
                                    print("[tilt] 重解失败: %s" % _e)
                            if _tilt_deg is not None and \
                                    (time.time() - solved.get("_tlt", 0.0)) >= 1.0:
                                solved["_tlt"] = time.time()
                                print("[tilt] ★实测装甲板倾角 = %+.2f°"
                                      "（0=竖直, >0=板顶后仰） | 上/下边宽比=%.4f | 像素宽=%.0fpx | 距离=%.2fm"
                                      % (_tilt_deg, _wh_ratio if _wh_ratio else 0.0,
                                         float(np.linalg.norm(np.array(pr[1]) - np.array(pr[0]))),
                                         float(np.linalg.norm(_t_c1))))
                            _gim_yaw = math.degrees(math.atan2(_t_c1[1], _t_c1[0]))      # 正=左
                            _gim_pitch = math.degrees(
                                math.atan2(_t_c1[2], math.hypot(_t_c1[0], _t_c1[1])))   # 抬头为正
                            target_yaw = _gim_yaw
                            target_pitch = _gim_pitch
                            # ★★★ 2026-10-13 补回弹道补偿（上一轮重构误删 ✗）★★★
                            #   用户问"为啥离远和离近 pitch 差不多" ⇒ 就是因为这段被删了：
                            #   peer 只发"目标相对炮口的仰角"，不含抬枪量 ⇒ 远近当然一样 ✗
                            #   距离优先用**仿真真值**（状态包 offset20 的 RangeCm）：
                            #     实测 PnP 的深度偏小（真值 2.87m / PnP 1.8m），用 PnP 距离会抬不够 ✓
                            _rng_sim = None
                            if bridge.range_cm is not None and bridge.range_cm > 1.0:
                                _rng_sim = bridge.range_cm / 100.0
                            _rng_pnp = float(np.linalg.norm(_t_c1))
                            _rng = _rng_sim if _rng_sim is not None else _rng_pnp
                            _se, _tof = ballistic_superelevation(_rng)
                            _se *= args.ballistic_scale
                            target_pitch += _se          # ← 抬枪角加在"相对仰角"上 ✓
                            _pnp_bear, _pnp_elev = _gim_yaw, _gim_pitch
                            if (time.time() - solved.get("_tfd", 0.0)) >= 1.0:
                                solved["_tfd"] = time.time()
                                _live_done()
                                print("[gim] cy=%+.2f cp=%+.2f | pos(云台系)=(%+.4f,%+.4f,%+.4f)"
                                      % (cy, cp, _pc[0], _pc[1], _pc[2]))
                                print("      目标(云台系,相对炮口): yaw=%+.2f pitch=%+.2f  |  距离=%.2fm"
                                      % (_gim_yaw, _gim_pitch, float(np.linalg.norm(_t_c1))))
                                print("      外参 xyz(云台系)=(%+.4f,%+.4f,%+.4f)"
                                      % (_t_cam[0], _t_cam[1], _t_cam[2]))
                                # ★★★ 用户要求：**同一帧并排对比** 视觉伺服(像素) 与 PnP 的 yaw ★★★
                                #   像素方位 = atan2(px_err, f)，取负后与 PnP 同为"正=左" ✓
                                _pxe = SHARED.get('px_err')
                                if _pxe is not None:
                                    _px_bear = -math.degrees(math.atan2(float(_pxe), F_PX))
                                    _diff = _gim_yaw - _px_bear
                                    _dot = "一致" if abs(_diff) < 1.0 else ("偏大" if abs(_diff) < 3.0 else "**严重不符**")
                                    print("[cmp] px_err=%+.0f | 视觉伺服方位=%+.2f° | PnP方位=%+.2f° | 差=%+.2f° %s"
                                          " | 距离=%.2fm"
                                          % (_pxe, _px_bear, _gim_yaw, _diff, _dot,
                                             float(np.linalg.norm(_t_c1))))

                            # ★ 记下"加偏置之前"的原始角（用于诊断：一眼分辨是链路错还是偏置在被加）
                            _raw_yaw = target_yaw
                            _raw_pitch = target_pitch
                            target_yaw += args.yaw_trim + AUTOBIAS['yaw']
                            target_pitch += args.pitch_trim + AUTOBIAS['pitch']
                            # ★★ 稳态误差积分（2026-10-12）★★
                            #   现象：两车都不动，从左边跟过去和从右边跟过去**停在不同的角度**。
                            #   原因：整条回路是纯比例控制（peer 误差×增益；仿真 Rate += -MouseYawRateCmd），
                            #         有摩擦/死区时误差小到推不动就停 ⇒ 残余误差取决于接近方向 ✓
                            #   修法：对**闭环角度残差**慢积分，叠加到目标上，把稳态误差压到 0。
                            #   只在像素误差很小（确实咬住目标）时积分，且限幅 3° 防跑飞 ✓
                            if args.integ:
                                _pe = SHARED.get("px_err")
                                _pev = SHARED.get("px_err_v")
                                _dtk = 1.0 / 60.0        # keeper 按 60 Hz 跑
                                if (_pe is not None and _pev is not None
                                        and abs(_pe) < INTEG['pxgate']
                                        and abs(_pev) < INTEG['pxgate']):
                                    # 闭环残差 = 目标角 − 炮口当前世界朝向（正=左）
                                    _res_y = target_yaw - (-cy)
                                    _res_p = target_pitch - cp
                                    INTEG['yaw'] = max(-INTEG['lim'], min(
                                        INTEG['lim'], INTEG['yaw'] + INTEG['rate'] * _res_y * _dtk))
                                    INTEG['pitch'] = max(-INTEG['lim'], min(
                                        INTEG['lim'], INTEG['pitch'] + INTEG['rate'] * _res_p * _dtk))
                                target_yaw += INTEG['yaw']
                                target_pitch += INTEG['pitch']
                            use_direct = (target_yaw, target_pitch,
                                          float(np.linalg.norm(_pc)))
                            pos_c = _pc          # 供下面的诊断日志使用
                except Exception as _e:
                    # ★ 2026-10-12：绝对角路径的异常**必须可见**。以前这里只 print 一行，
                    #   而 NameError 之类的编程错误会让整条路径静默失效（"改了没反应"）✗
                    #   现在打印完整 traceback，一眼能看出是哪一行炸的 ✓
                    import traceback as _tb
                    _live_done()
                    print("[tf] EXCEPTION in absolute-angle path:")
                    _tb.print_exc()
                use_yp = None
                try:
                    d0 = (solved.get("dets") or [None])[0]
                    st0 = bridge.last_state
                    if d0 is not None and st0 is not None and YAW_PNP:
                        import yaw_pnp as _yp
                        pts_c = np.asarray(d0[0], float).reshape(4, 2) - np.array([CAM_W / 2.0, CAM_H / 2.0])
                        cy, cp, cr = st0[15], st0[16], st0[17]      # 相机 rpy（仿真世界系，度 ✓）
                        cam_rpy = np.deg2rad([cr, cp, cy])           # (roll, pitch, yaw)
                        cam_pos = np.array(st0[12:15], float) / 100.0
                        yaw_w, pos_w, info2 = _yp.solve_yaw(
                            K, pts_c, PLATE_W, PLATE_H, cam_rpy, cam_pos,
                            armor_tilt_deg=ARMOR_TILT_DEG, yaw_center=cam_rpy[2])
                        if yaw_w is not None:
                            dy = math.hypot(pos_w[0] - cam_pos[0], pos_w[1] - cam_pos[1])
                            # ★ 守卫（verify_yawpnp.py 实测 ✓）：移植版在部分离轴/远距样本上
                            #   三分搜索会跑到**搜索边界**（±90° ⇒ 输出 89.79 ✗）—— 那是退化解 ✓。
                            #   命中边界就**不采用**我们的值，退回他们的 yaw ✓（宁可保守 ✗ 也不喂垃圾 ✓）。
                            rel = abs(math.degrees(((yaw_w - cam_rpy[2] + math.pi) % (2 * math.pi)) - math.pi))
                            if rel < 80.0:
                                use_yp = (math.degrees(yaw_w),
                                          math.degrees(math.atan2(pos_w[2] - cam_pos[2], max(dy, 1e-6))), dy)
                            else:
                                print("[yawpnp] 搜索到边界(%.1f deg) -> 本帧不采用我们的 yaw ✓" % math.degrees(yaw_w))
                except Exception as _e:
                    print("[yawpnp] err", _e)
                # 绝对角通道：直接把他们的 ShotSolution 角度转成我们仿真的量纲发下去。
                # yaw_sign 默认 -1（他们逆时针为正 / 仿真顺时针为正，已双重验证）；
                # pitch 双方都是抬头为正（sign +1）。
                ayaw = args.yaw_sign * math.degrees(sol.yaw)
                # [DATA FIX 2026-10-12] ???? pitch ????? ?
                #   ???pnp_20261012_115104.csv ?250??????? 320 ???
                #     corr(vision_pitch, px_err_v) = -0.67   corr(rel_pitch, px_err_v) = -0.81
                #     d(rel_pitch)/d(px_err_v) = -0.080  ? ?????????????? ?
                #   ? ???? ???????? pitch ??????? ??
                apitch = -args.pitch_sign * math.degrees(sol.pitch)
                # ===== pitch ???????2026-10-12????"????????" ??=====
                #   ????????????????? target_pitch ??**????** ?
                #         ? ???????????????? ? ??????0 ? ????? ??
                if use_direct is not None:
                    _pxv = 0.0
                    _st2 = bridge.last_state
                    if _st2 is not None and SHARED.get("px_err_v") is not None:
                        _pxv = SHARED["px_err_v"]
                    # ★★ 2026-10-12：PITCH_BIAS / YAW_BIAS 的"自学习"已停用 ★★
                    #   那两个积分器是给**旧 TF 公式**打补丁用的（旧公式算出的 yaw 有
                    #   十几度常数偏、pitch 有十几度常数偏，于是用"慢慢把差值学掉"来兜）。
                    #   现在 TF 已经改成真值精确的算法（peer_tf_selftest 实测误差 0.0°），
                    #   再自学习就会**把正确的量再抠掉一块** ⇒ 云台稳定地偏一个常数角 ✗。
                    #   停用后：raw 量直接进控制环。要重新启用设 BIAS_ADAPT = True。
                    _cur_elev = None
                    if _st is not None:
                        _cur_elev = _st[10]                     # 云台仰角（度，抬头为正）
                    # ★★★ 2026-10-12 修正（上次改反了）★★★
                    #   peer 走的是 **AbsCode == 1** 通道 ⇒ 对应仿真的
                    #       else if (bUseAbsolute) { ... }
                    #   那个分支（WLVehiclePawn.cpp:7393-7395）是：
                    #       WantRot   = RotFromElevation(FiltPitchDeg)     ← FiltPitchDeg 当**绝对仰角**用
                    #       PitchErr  = WantRot - Gimbal.Pitch
                    #   ⇒ pitch 必须发**绝对仰角**，不能减当前仰角 ✗
                    #   （"增量"语义属于 AbsCode==2 那个分支的 WantElev2 = elev_now + AbsPitchDeg，
                    #     而 peer 从没发过 code=2。上次据 code=2 改成增量是错的 ✓）
                    # yaw 同样是绝对语义（WantYawAbs − CurWorldYaw）✓
                    use_direct = (use_direct[0], use_direct[1], use_direct[2])
                    if BIAS_ADAPT:
                        PITCH_BIAS[0] += 0.01 * (use_direct[1] - PITCH_BIAS[0])
                    # ★★★ 2026-10-12：**TF_FILT 这层滤波已删除** ★★★
                    #   它是给旧公式（输出抖 ±40°）打的补丁：yaw 变化 >20° 丢弃、pitch >12° 丢弃，
                    #   其余按 0.35 / 0.25 一阶平滑。后果是**调偏置几乎无效** ✗：
                    #   按一次方向键 ±0.1°，过门限后只挪 0.035°，再被冲掉 ⇒ 用户实测"改了没反应" ✓
                    #   TF 现在已经准了（残差 ~1.7°），这层滤波没必要，直接下发原始绝对角。
                    #   ⚠ 若以后又出现"检测抖导致云台抖"，**先查检测/求解器**，不要把这层平滑加回来 ✗
                    use_direct = (use_direct[0], use_direct[1], use_direct[2])
                    HOLD['yaw'], HOLD['pitch'], HOLD['dist'] = use_direct[0], use_direct[1], use_direct[2]
                    use_direct_sent = True
                    # 每帧的对照日志：TF 绝对角 vs 像素真值角（两者应一致）
                    _pxe = SHARED.get('px_err') or 0.0
                    _pxv = SHARED.get('px_err_v') or 0.0
                    _pb = math.degrees(math.atan2(_pxe, F_PX))
                    _pv = math.degrees(math.atan2(-_pxv, F_PX))     # 抬头为正
                    # ★★ 2026-10-12 诊断加严：把内部量全打出来 ★★
                    #   判据：像素方位 = px_err(右为正) -> 像素角 -_pb(左为正)。
                    #   目标的**绝对**方位 ≈ 相机世界偏航 cy + 像素角(-_pb)。
                    #   TF 给的 target_yaw 应该 ≈ 这个值（差 ≤ 几度）。
                    #   若差几十度，而 pos_c 方向与像素方位不符 ⇒ PnP/角点约定问题；
                    #   若 pos_c 方向对但 target_yaw 不对 ⇒ 我的换算式问题。
                    _pc = np.asarray(pos_c, float)
                    _pcy = math.degrees(math.atan2(_pc[1], _pc[0])) if _pc is not None else 0.0
                    _expect_abs = cy - _pb
                    # ★★ 2026-10-12 一次性取证（只打前 5 次检测，之后自动闭嘴）★★
                    #   目的：把 pos_c 的**原始分量**、相机姿态、像素误差同时打出来，
                    #   用一次真实运行就能判定 pos_c 是"相机系(C1)"还是"绝对系(C')"，
                    #   从此不再靠反推/合成数据猜。
                    if solved.get("_dbgn", 0) < 5:
                        solved["_dbgn"] = solved.get("_dbgn", 0) + 1
                        _live_done()
                        print("[probe#%d] pose_src=%s" % (solved["_dbgn"], _pose_src))
                        print("   cam_rpy(deg, r/p/y 正=左) = (%+.3f, %+.3f, %+.3f)"
                              % (cr, cp, cy))
                        print("   cam_pos(m)               = (%+.4f, %+.4f, %+.4f)"
                              % (cam_pos_[0], cam_pos_[1], cam_pos_[2]))
                        print("   px_err / px_err_v (px)   = (%+.1f, %+.1f)"
                              % (SHARED.get('px_err') or 0.0, SHARED.get('px_err_v') or 0.0))
                        print("   pos_c RAW                = (%+.5f, %+.5f, %+.5f)  |v|=%.4f"
                              % (_pc[0], _pc[1], _pc[2], float(np.linalg.norm(_pc))))
                        _Rdbg = rpy_to_R(cam_rpy)
                        _vw = _Rdbg @ _pc
                        print("   R@pos_c                  = (%+.5f, %+.5f, %+.5f)"
                              % (_vw[0], _vw[1], _vw[2]))
                        _vt = _Rdbg.T @ _pc
                        print("   R.T@pos_c                = (%+.5f, %+.5f, %+.5f)"
                              % (_vt[0], _vt[1], _vt[2]))
                        print("   -> yaw/pitch: RAW %+.2f/%+.2f | R@ %.2f/%.2f | R.T@ %.2f/%.2f"
                              % (math.degrees(math.atan2(_pc[1], _pc[0])),
                                 math.degrees(math.atan2(_pc[2], math.hypot(_pc[0], _pc[1]))),
                                 math.degrees(math.atan2(_vw[1], _vw[0])),
                                 math.degrees(math.atan2(_vw[2], math.hypot(_vw[0], _vw[1]))),
                                 math.degrees(math.atan2(_vt[1], _vt[0])),
                                 math.degrees(math.atan2(_vt[2], math.hypot(_vt[0], _vt[1])))))
                        print("   （对照：视觉自己的解 theirs yaw=%+.2f pitch=%+.2f / dist=%.2f）"
                              % (math.degrees(sol.yaw), math.degrees(sol.pitch),
                                 float(np.linalg.norm(_pc))))
                    if args.verbose and (time.time() - solved.get("_rawt", 0.0)) >= 0.5:
                        solved["_rawt"] = time.time()
                        print("[tf] RAW px=(%+.0f,%+.0f) n=%.1f | pxbearing yaw=%+.2f (abs≈%+.2f) | "
                              "pos_c=(%+.3f,%+.3f,%+.3f) dist=%.2f | POSC yaw=%+.2f pitch=%+.2f | "
                              "cam rpy=(%+.1f,%+.1f,%+.1f) pos=(%+.2f,%+.2f,%+.2f) | "
                              "OUT yaw=%+.2f pitch=%+.2f | theirs yaw=%+.1f pitch=%+.1f"
                              % (pr[:, 0].mean(), pr[:, 1].mean(), pr[:, 0].max() - pr[:, 0].min(),
                                 _pb, _expect_abs,
                                 _pc[0], _pc[1], _pc[2], float(np.linalg.norm(_pc)),
                                 _pcy, math.degrees(math.atan2(_pc[2], math.hypot(_pc[0], _pc[1]))),
                                 cy, cp, cr, cam_pos_[0], cam_pos_[1], cam_pos_[2],
                                 use_direct[0], use_direct[1],
                                 math.degrees(sol.yaw), math.degrees(sol.pitch)))
                    ayaw, apitch = use_direct[0], use_direct[1]
                    # ★★ 2026-10-12 取证（唯一目的：一次性判定坐标系/符号到底哪一步错）★★
                    #   用同一个 UDP 包里带来的**相机位姿**与**像素**做"投影反算"：
                    #     像素推算出的目标方向 ≈ (相机朝向) + (图像里的方位)
                    #   再和 pos_c 的各种可能解释对比，谁对上谁就是正确坐标系 ✓
                    #   日志是**事件性**的（每秒 2 条），不会刷屏。
                    if (time.time() - solved.get("_probet", 0.0)) >= 0.5:
                        solved["_probet"] = time.time()
                        _live_done()
                        _R = rpy_to_R(cam_rpy)
                        _vw = _R @ _pc            # 一种解释：pos_c 在相机系
                        _pxe2 = SHARED.get('px_err')
                        _pxv2 = SHARED.get('px_err_v')
                        # 像素真值（正=左 / 抬头为正）
                        _pix_yaw_abs = (-cy) - math.degrees(math.atan2(_pxe2 or 0.0, F_PX))
                        _pix_pit_abs = cp + math.degrees(math.atan2(-(_pxv2 or 0.0), F_PX))
                        def _ang(v):
                            return (math.degrees(math.atan2(v[1], v[0])),
                                    math.degrees(math.atan2(v[2], math.hypot(v[0], v[1]))))
                        _a_raw = _ang(_pc)
                        _a_rot = _ang(_vw)
                        print("[probe] src=%s | cam(yaw=%+.2f pitch=%+.2f roll=%+.2f)"
                              % (_pose_src, cy, cp, cr))
                        print("        px=(%s,%s)  -> 像素推算 yaw=%+.2f pitch=%+.2f"
                              "   ★注意：这是**我自己的推导量，不是仿真真值**，"
                              "判对错只看仿真的 WLAIMTRUE/TRUE_abs"
                              % (("%+.0f" % _pxe2) if _pxe2 is not None else "n/a",
                                 ("%+.0f" % _pxv2) if _pxv2 is not None else "n/a",
                                 _pix_yaw_abs, _pix_pit_abs))
                        print("        pos_c=(%+.4f,%+.4f,%+.4f) |v|=%.3f -> 原样 yaw=%+.2f pitch=%+.2f"
                              % (_pc[0], _pc[1], _pc[2], float(np.linalg.norm(_pc)),
                                 _a_raw[0], _a_raw[1]))
                        print("        R@pos_c=(%+.4f,%+.4f,%+.4f)            -> 旋转 yaw=%+.2f pitch=%+.2f"
                              % (_vw[0], _vw[1], _vw[2], _a_rot[0], _a_rot[1]))
                        print("        误差: 原样 dy=%+.2f dp=%+.2f | 旋转 dy=%+.2f dp=%+.2f"
                              % (_a_raw[0] - _pix_yaw_abs, _a_raw[1] - _pix_pit_abs,
                                 _a_rot[0] - _pix_yaw_abs, _a_rot[1] - _pix_pit_abs))
                        print("        下发: OUT yaw=%+.2f pitch(增量)=%+.2f rng(sim)=%s | 弹道=%+.2f"
                              % (use_direct[0], use_direct[1],
                                 ("%.2fm" % (bridge.range_cm / 100.0)) if bridge.range_cm else "n/a",
                                 _se * args.ballistic_scale))
                    # ★ 2026-10-12：改成**单行原地刷新**（\r，不换行）—— 不再刷屏，
                    #   调偏置那行永远留在屏幕底部 ✓
                    if (time.time() - solved.get("_statust", 0.0)) >= 0.25:
                        solved["_statust"] = time.time()
                        # ★ 这两个量原本在被我重写的 probe 块里定义过；重写后必须在这里补回来，
                        #   否则下面引用会 NameError ⇒ 被外层 except 吞掉 ⇒ **整条绝对角路径静默失效** ✗
                        _p2 = SHARED.get("px_err")
                        _pk = -(math.degrees(math.atan2(_p2, F_PX)) if _p2 is not None else 0.0)
                        _camhead = -cy
                        _d = FMathUnwind(use_direct[0] - (_camhead + _pk))
                        # 按键反馈直接显示在这**唯一**一条状态行上（不再另起一行）✓
                        _adj = ""
                        if time.time() - ADJUST['t'] < 2.5:
                            _adj = " [%s]" % ADJUST['tag']
                        _live("[tuner] bias y=%+.2f p=%+.2f%s | OUT %+.2f/%+.2f | bal %+.2f@%.1fm(sim%s) | I %+.2f/%+.2f | px %s | %s"
                              % (AUTOBIAS['yaw'], AUTOBIAS['pitch'], _adj,
                                 use_direct[0], use_direct[1],
                                 _se * args.ballistic_scale, _rng,
                                 "" if (_rng_sim is not None) else "?pnp%.1f" % _rng_pnp,
                                 INTEG['yaw'], INTEG['pitch'],
                                 ("%+.0f" % _p2) if _p2 is not None else "n/a",
                                 _pose_src))
                    if args.verbose and _n2 % 15 == 1:
                        _doff = ""
                        _fp3 = solved.get("frame_pose") or bridge.frame_pose
                        if _fp3 is not None and len(_fp3) >= 9:
                            _doff = " doff=(%+.3f,%+.3f,%+.3f)m" % (_fp3[6], _fp3[7], _fp3[8])
                        print("\n[abs] pose=%-11s cam_head=%+.2f cam_pitch=%+.2f roll=%+.2f%s | "
                              "px=%s | PK_bear=%+.2f PK_pitch=%+.2f | OUT=%+.2f/%+.2f rng=%.2f | "
                              "cam_head+PK=%+.2f (应≈OUT) | d=%+.2f | theirs=%+.1f/%+.1f"
                              % (_pose_src, _camhead, cp, cr, _doff,
                                 ("%+.0f" % _p2) if _p2 is not None else "n/a", _pk, _pv2,
                                 use_direct[0], use_direct[1], use_direct[2],
                                 _camhead + _pk,
                                 FMathUnwind(use_direct[0] - (_camhead + _pk)),
                                 math.degrees(sol.yaw), math.degrees(sol.pitch)))
                if use_yp is not None:
                    print("[yawpnp] theirs yaw=%+.1f pitch=%+.1f | OURS yaw=%+.1f pitch=%+.1f dist=%.2fm"
                          % (math.degrees(sol.yaw), math.degrees(sol.pitch), use_yp[0], use_yp[1], use_yp[2]))
                    ayaw, apitch = use_yp[0], use_yp[1]
                # [FIX 2026-10-12] ??? ? ???????????? ?????????? ? ?? ??
                if (not use_direct_sent) and HOLD['yaw'] is not None:
                    ayaw, apitch = HOLD['yaw'], HOLD['pitch']
                # ===== 2026-10-12：像素伺服（lag-immune）=====
                # 为什么回到增量通道：绝对角通道的前提是"给出目标的世界朝向"，而世界朝向
                # 必须由"图像时刻的相机姿态"推出。实测现场两者**对不上**（同一帧里
                # cam rpy=(61.7,0,-11.0) 而检测框在画面右 px=+67 ⇒ 相机朝向与图像矛盾），
                # 于是绝对角整帧错几十度 ⇒ 云台永远朝一个方向转到底 ✓（用户实测现象）。
                # 这条链上**唯一不会撒谎的量是像素**：目标在画面右就右转，不依赖任何姿态、
                # 不依赖时间对齐 ✓（也正是用户老代码 aim_peer.py 稳定的原因）。
                # 增量语义（仿真端 useAbsolute=0 分支）：
                #     YawVel = dYaw 度/帧 -> MouseYawRateCmd += YawVel  （正 = 炮口右）
                #     PitchVel = dPitch 度/帧（正 = 炮口上抬）
                if args.mode == "pixel":
                    _p = SHARED.get("px_err")
                    _pv = SHARED.get("px_err_v")
                    if _p is not None and _pv is not None:
                        # 目标在画面右(px>0) ⇒ 必须右转 ⇒ dYaw>0 ✓
                        dyaw = args.pixel_gain * _p / (F_PX / 100.0)
                        dpitch = -args.pixel_gain * _pv / (F_PX / 100.0)
                        dyaw = max(-args.pixel_max_step, min(args.pixel_max_step, dyaw))
                        dpitch = max(-args.pixel_max_step, min(args.pixel_max_step, dpitch))
                        bridge.send_cmd(dyaw, dpitch, bool(sol.fire), True, time.time(),
                                        dets_for_echo=solved.get('dets'),
                                        echo_frame_id=solved.get('last_fid', 0))
                        _n = solved.get("_pixn", 0) + 1
                        solved["_pixn"] = _n
                        if _n % 30 == 1:
                            print("[pix] px=(%+.0f,%+.0f) -> dYaw=%+.2f dPitch=%+.2f deg/frame | "
                                  "their yaw=%+.1f pitch=%+.1f"
                                  % (_p, _pv, dyaw, dpitch, math.degrees(sol.yaw),
                                     math.degrees(sol.pitch)))
                    last_sent_n = solved["n"]
                    time.sleep(1.0 / 60.0)
                    continue
                bridge.send_cmd(0.0, 0.0, bool(sol.fire), True, time.time(),
                                abs_yaw=ayaw, abs_pitch=apitch,
                                dets_for_echo=solved.get('dets'),
                                echo_frame_id=solved.get('last_fid', 0))
        except Exception as e:                             # noqa
            print("[keeper] %s: %s" % (type(e).__name__, e), file=sys.stderr)
        time.sleep(1.0 / 60.0)


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


if __name__ == "__main__":
    main()
