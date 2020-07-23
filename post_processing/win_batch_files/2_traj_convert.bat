@ECHO OFF
setlocal EnableExtensions EnableDelayedExpansion

set /a traj_num=0
set traj=none

for %%i in (*.traj) do (
  set traj=%%~nxi
  set /a traj_num=!traj_num!+1
)

if %traj_num%==1 (
python C:\OpenMMS\code\openmms_traj_convert.py %traj%
)

if %traj_num%==0 echo No TRAJ file was found in the current directory

if %traj_num% gtr 1 echo More than one TRAJ file was found in the current directory, only a single TRAJ file should be present

PAUSE