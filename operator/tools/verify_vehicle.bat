@echo off
setlocal
set EXE=D:\xuhuan\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe
set PROJ=D:\UEproject\RM2019Sim 4.27\RM2019Sim.uproject
set LOG=D:\DeepseekPlungin\rm2019_tools\verify_vehicle.log
"%EXE%" "%PROJ%" -nullrhi -unattended -nosplash -NoLogTimes -abslog="%LOG%" -ExecCmds="py D:/DeepseekPlungin/rm2019_tools/verify_vehicle_in_level.py"
endlocal
