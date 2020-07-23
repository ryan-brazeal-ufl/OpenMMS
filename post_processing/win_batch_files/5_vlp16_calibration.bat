@ECHO OFF
setlocal EnableExtensions EnableDelayedExpansion

set do_vlp16_internal_cal=False
set do_extra_bore_cal=False
set export_final_ECEF_planes=False

python C:\OpenMMS\code\openmms_vlp16_calibration.py %cd% %do_vlp16_internal_cal% %do_extra_bore_cal% %export_final_ECEF_planes% 

PAUSE