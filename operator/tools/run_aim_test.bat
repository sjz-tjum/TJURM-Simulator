@echo off
rem ============================================================================
rem  run_aim_test.bat —— 双击就能跑的自瞄联调（不用记任何命令、不用开控制台）
rem
rem  做三件事：
rem    1) 清掉上次残留的 python 视觉端（残留进程会占着 41001 端口，新 peer 一帧都收不到）
rem    2) 开一个窗口跑视觉端 aim_peer.py（跑你的 v5n 模型）
rem    3) 起仿真：开自瞄桥 + 靶车摆在正前方 2.5 m
rem
rem  进游戏后：
rem    N = 开/关 自瞄相机画面（右上角，模型看到的那张图）
rem    O = 开/关 视觉链路（正常不用按，脚本已经开了；画面会显示 link=UP cmds=N）
rem    P = 光圈循环 -0.5 / -1.0 / 0 EV（场馆灯不受影响）
rem    U = 靶车距离循环 1.5 / 2.5 / 4 m
rem    按住鼠标右键 = 进自瞄（云台交给视觉端），松开 = 手动
rem    J = 录一段自瞄相机画面给我（存到 Saved\AimShots\rec_<时间戳>\）
rem ============================================================================

set PROJ=D:\UEproject\RM2019Sim 4.27\RM2019Sim.uproject
set UE=D:\xuhuan\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe
set PEER=D:\UEproject\RM2019Sim 4.27\Tools\aim_peer.py
set MODEL=C:\Users\ASUS\Desktop\北洋机甲\neoautoaim1\TJURM-Aiming\config\models\v5n_7c_blk_640_480_v2.onnx

echo [1/3] 清理残留的 python 视觉端...
taskkill /F /IM python.exe >nul 2>&1

echo [2/3] 启动视觉端（新窗口，会一直打印心跳）...
rem  -u 必须加：Python 的 stdout 默认块缓冲，管道/后台跑时 print 不出来，
rem  用户会以为"视觉端没有输出"（实测踩过）。
rem  不要再传 --gain/--pitch-gain：调好的默认值在脚本里（gain 0.70 / pitch 0.45 / 偏置+弹道解算），
rem  命令行覆盖会把它们打回旧值 ✗。
start "AIM PEER" cmd /k python -u "%PEER%" --dump 40

echo [3/3] 启动仿真（自瞄桥 + 靶车 2.5 m）...
timeout /t 2 /nobreak >nul
start "" "%UE%" "%PROJ%" /Game/Maps/RM_Arena -game -windowed -ResX=1280 -ResY=720 -nosplash -NoLogTimes -WLAimNet -WLAimRange=250

echo.
echo 全部起来了。进游戏后：按住右键 = 自瞄；N 看自瞄画面；J 录一段。
pause
