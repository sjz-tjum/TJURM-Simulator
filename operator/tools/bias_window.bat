@echo off
REM 双击这个就能开"调偏置窗口"。先在任务管理器里结束所有 python.exe，避免抢 41001 端口。
cd /d "%~dp0"
python -X utf8 bias_window.py %*
pause
