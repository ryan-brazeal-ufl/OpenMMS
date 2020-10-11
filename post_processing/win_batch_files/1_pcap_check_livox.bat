@ECHO OFF
setlocal EnableExtensions EnableDelayedExpansion

set /a pcap_num=0
set /a livox_num=0
set pcap=none
set livox=none

for %%i in (*.pcap) do (
  set pcap=%%~nxi
  set /a pcap_num=!pcap_num!+1
)

for %%i in (*.livox) do (
  set livox=%%~nxi
  set /a livox_num=!livox_num!+1
)

if %pcap_num%==1 (
if %livox_num%==1 (
python C:\OpenMMS\code\openmms_pcap_check_livox.py %pcap% %livox%
)
)

if %pcap_num%==0 echo No PCAP file was found in the current directory

if %pcap_num% gtr 1 echo More than one PCAP file was found in the current directory, only a single PCAP file should be present

if %livox_num%==0 echo No LIVOX file was found in the current directory

if %livox_num% gtr 1 echo More than one LIVOX file was found in the current directory, only a single LIVOX file should be present


PAUSE