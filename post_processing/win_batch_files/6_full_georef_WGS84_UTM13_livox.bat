@ECHO OFF
setlocal EnableExtensions EnableDelayedExpansion

set boresight_file=zeros.txt
set output_file=full_pc.laz
set start_proc_time=-1
set end_proc_time=-1
set quick_processing=False
set verbose_output=False
set display_plots=False
set time_sync_corr=0
set geo_cs=4326
set proj_cs=32613

set /a counter1=0
set /a counter2=0
set /a counter3=0
set pcap=none
set nav=none
set livox=none

for %%i in (*.pcap) do (
  set pcap=%%~nxi
  set /a counter1=!counter1!+1
)

for %%i in (*.txt) do (
  set nav=%%~nxi
  set /a counter2=!counter2!+1
)

for %%i in (*.livox) do (
  set livox=%%~nxi
  set /a counter3=!counter3!+1
)

if %counter1%==1 (
if %counter2%==1 (
if %counter3%==1 (
python C:\OpenMMS\code\openmms_georeference_livox.py C:\OpenMMS\code\serf\geo\ C:\OpenMMS\sys_params\%boresight_file% %nav% %pcap% %livox% %output_file% %quick_processing% %verbose_output% %start_proc_time% %end_proc_time% %display_plots% %geo_cs% %proj_cs% %time_sync_corr% %cd%
)
)
)

if %counter1%==0 echo No PCAP file was found in the current directory

if %counter1% gtr 1 echo More than one PCAP file was found in the current directory, only a single PCAP file should be present

if %counter2%==0 echo No NAV text (.txt) file was found in the current directory

if %counter2% gtr 1 echo More than one NAV text (.txt) file was found in the current directory, only a single .txt file should be present

if %counter3%==0 echo No LIVOX file was found in the current directory

if %counter3% gtr 1 echo More than one LIVOX file was found in the current directory, only a single LIVOX file should be present

PAUSE