@ECHO OFF
setlocal EnableExtensions EnableDelayedExpansion

set /a pcap_num=0
set pcap=none

for %%i in (*.pcap) do (
  set pcap=%%~nxi
  set /a pcap_num=!pcap_num!+1
)

if %pcap_num%==1 (
python C:\OpenMMS\code\openmms_pcap_check.py %pcap%
)

if %pcap_num%==0 echo No PCAP file was found in the current directory

if %pcap_num% gtr 1 echo More than one PCAP file was found in the current directory, only a single PCAP file should be present

PAUSE