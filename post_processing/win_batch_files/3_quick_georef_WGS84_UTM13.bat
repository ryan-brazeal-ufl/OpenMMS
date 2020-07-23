@ECHO OFF
setlocal EnableExtensions EnableDelayedExpansion

set boresight_file=zeros.txt
set output_file=quick_pc.las
set start_proc_time=-1
set end_proc_time=-1
set thin_factor=1
set quick_processing=True
set verbose_output=False
set display_plots=False
set use_vlp16_calibration=True
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
python C:\OpenMMS\code\openmms_georeference.py C:\OpenMMS\code\serf\geo\ C:\OpenMMS\sys_params\%boresight_file% %nav% %pcap% %output_file% %quick_processing% %verbose_output% %start_proc_time% %end_proc_time% %thin_factor% %display_plots% %geo_cs% %proj_cs% %cd% %use_vlp16_calibration%
)
)

if %counter1%==0 echo No PCAP file was found in the current directory

if %counter1% gtr 1 echo More than one PCAP file was found in the current directory, only a single PCAP file should be present

if %counter2%==0 echo No NAV text (.txt) file was found in the current directory

if %counter2% gtr 1 echo More than one NAV text (.txt) file was found in the current directory, only a single .txt file should be present

PAUSE