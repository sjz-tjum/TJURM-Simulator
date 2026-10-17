# bias_window.py - 在游戏里实时调偏置的小窗口
#
# 直接双击/运行这个脚本即可。它会：
#   1) 把 peer 拉起来（如果已经在跑会提示先关掉）
#   2) 开一个**可见的窗口**，里面实时显示当前偏置
#   3) 用键盘方向键直接改偏置 —— 改完**下一帧就生效**，不用重启任何东西
#
# 按键：
#     ← →        yaw  −/+ 0.1°      （正 = 往左补）
#     ↑ ↓        pitch +/− 0.1°      （正 = 往抬头补）
#     PgUp PgDn  pitch +/− 1.0°      （粗调，你说 pitch 可以大胆给）
#     b          自动配平：采 ~1.5s 的均值算出一个偏置（推荐先用这个）
#     r          清零
#     y          保存到 bias_saved.json（下次启动自动载入）
#     ?          帮助
#
# 正负号怎么记：
#     弹着点在板心**右侧** -> 要往左补 -> 按 ← 或 → 让 yaw 变**正**
#     弹着点在板心**上方** -> 要往低压   -> 按 ↓ 让 pitch 变**负**
import os
import subprocess
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
PEER = os.path.join(TOOLS, "plugyou_peer.py")

if not os.path.exists(PEER):
    print("找不到 %s" % PEER)
    sys.exit(1)

# 顺手看一下有没有别的 peer 在跑（同端口会抢包）—— 上面已经自动清掉了，这里只做确认
try:
    out = subprocess.run(["netstat", "-ano", "-p", "UDP"],
                         capture_output=True, text=True).stdout
    busy = [ln for ln in out.splitlines() if "127.0.0.1:41001" in ln]
    if busy:
        print("注意：41001 仍被占用（%s），可能清理没成功。" % busy[0].strip())
except Exception:
    pass

# ---- 先把占着 41001 的旧 peer 杀掉（否则两边抢同一个 UDP 包，两个都用不了）----
def _kill_stale_peers():
    """只杀**命令行里带 plugyou_peer.py / bias_window.py** 的 python，不动别的 python。"""
    try:
        out = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | "
             "Where-Object { $_.CommandLine -match 'plugyou_peer|bias_window' } | "
             "Select-Object -ExpandProperty ProcessId"],
            capture_output=True, text=True, timeout=20).stdout
    except Exception as e:
        print("（检查旧进程失败：%s）" % e)
        return
    pids = [int(x) for x in out.split() if x.strip().isdigit()]
    pids = [p for p in pids if p != os.getpid()]
    if not pids:
        return
    print("发现旧的 peer 进程：%s —— 先关掉它们（否则会抢 41001 包）" % pids)
    for p in pids:
        try:
            subprocess.run(["taskkill", "/PID", str(p), "/F"],
                           capture_output=True, timeout=10)
        except Exception:
            pass
    import time
    time.sleep(2.0)
    print("已清理，继续启动新的。\n")


_kill_stale_peers()

print("启动 peer（bias 窗口）…")
print("参数：--comp 0（枪口补偿关），偏置全部由这个窗口控制\n")

os.chdir(TOOLS)
cmd = [sys.executable, "-X", "utf8", "-u", PEER, "--comp", "0"] + sys.argv[1:]
# ★ 给 peer 开一个**独立的新控制台**：这样它自己就是那个"调偏置的窗口"，
#   方向键直接作用在它身上（如果共用本窗口，两个进程抢键盘会互相打架）
CREATE_NEW_CONSOLE = 0x00000010
proc = subprocess.Popen(cmd, cwd=TOOLS, creationflags=CREATE_NEW_CONSOLE)

print("peer PID = %d" % proc.pid)
print("-" * 66)
print("★ 调偏置在那个**新开的黑色窗口**里操作（不是这个）：")
print("     ← →        yaw  −/+ 0.1°")
print("     ↑ ↓        pitch +/− 0.1°")
print("     PgUp/PgDn  pitch +/− 1.0°")
print("     b          自动配平（推荐先用）")
print("     r 清零   y 保存   ? 帮助")
print("-" * 66)
try:
    proc.wait()
except KeyboardInterrupt:
    proc.terminate()
