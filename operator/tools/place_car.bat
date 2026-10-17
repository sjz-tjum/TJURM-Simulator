@echo off
REM Place the WL_Vehicle pawn into RM_Arena (defect 1: the car must be visible without pressing Play).
REM
REM Runs the FULL editor headless, not -run=PythonScript: that commandlet has no level editor and
REM EditorLevelLibrary.load_level() dereferences it and crashes.  The editor loads
REM EditorStartupMap = /Game/Maps/RM_Arena by itself, so the script only edits the open level.
REM
REM The editor does NOT exit when the ExecCmd finishes, so the caller polls the log for the
REM script's SAVED line and then stops the process.
setlocal
set EXE=D:\xuhuan\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe
set PROJ=D:\UEproject\RM2019Sim 4.27\RM2019Sim.uproject
set LOG=D:\DeepseekPlungin\rm2019_tools\place_vehicle.log
"%EXE%" "%PROJ%" -nullrhi -unattended -nosplash -NoLogTimes -abslog="%LOG%" ^
  -ExecCmds="py D:/DeepseekPlungin/rm2019_tools/place_vehicle_in_level.py"
endlocal
