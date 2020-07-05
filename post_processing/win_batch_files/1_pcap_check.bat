@ECHO OFF
setlocal EnableExtensions EnableDelayedExpansion

set /a counter1=0
set pcap=none

for %%i in (*.pcap) do (
  set pcap=%%~nxi
  set /a counter1=!counter1!+1
)

if %counter1%==1 (
python C:\OpenMMS\code\openmms_pcap_check.py %pcap%
)

if %counter1%==0 echo No PCAP file was found in the current directory

if %counter1% gtr 1 echo More than one PCAP file was found in the current directory, only a single PCAP file should be present

PAUSE