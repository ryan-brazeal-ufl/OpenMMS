@ECHO OFF
setlocal EnableExtensions EnableDelayedExpansion

set /a counter1=0
set traj=none

for %%i in (*.traj) do (
  set traj=%%~nxi
  set /a counter1=!counter1!+1
)

if %counter1%==1 (
python C:\OpenMMS\code\openmms_traj_convert.py %traj%
)

if %counter1%==0 echo No TRAJ file was found in the current directory

if %counter1% gtr 1 echo More than one TRAJ file was found in the current directory, only a single TRAJ file should be present

PAUSE