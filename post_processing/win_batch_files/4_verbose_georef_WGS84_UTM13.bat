@ECHO OFF
setlocal EnableExtensions EnableDelayedExpansion
echo Please wait...


set boresight_file=zeros.txt

set outputFile=verbose_pc.las

set start_proc_time=-1

set end_proc_time=-1

set thin_factor=1


set quick_processing=False

set verbose_output=True

set display_plots=True

set useVLP16Calibration=True

set geo_cs=4326

set proj_cs=32613



set /a counter1=0
set /a counter2=0
set pcap=none
set nav=none

for %%i in (*.pcap) do (
  set pcap=%%~nxi
  set /a counter1=!counter1!+1
)

for %%i in (*.txt) do (
  set nav=%%~nxi
  set /a counter2=!counter2!+1
)

if %counter1%==1 (
if %counter2%==1 (
python C:\OpenMMS\code\openmms_georeference.py C:\OpenMMS\code\serf\geo\ C:\OpenMMS\sys_params\%boresight_file% %nav% %pcap% %outputFile% %quick_processing% %verbose_output% %start_proc_time% %end_proc_time% %thin_factor% %display_plots% %geo_cs% %proj_cs% %cd% %useVLP16Calibration%
)
)

if %counter1%==0 echo No PCAP file was found in the current directory

if %counter1% gtr 1 echo More than one PCAP file was found in the current directory, only a single PCAP file should be present

if %counter2%==0 echo No NAV text (.txt) file was found in the current directory

if %counter2% gtr 1 echo More than one NAV text (.txt) file was found in the current directory, only a single .txt file should be present

PAUSE