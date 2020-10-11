@ECHO OFF

set max_images=500
set orientation_only=True

python C:\OpenMMS\code\openmms_camera_calibration.py %cd% %max_images% %orientation_only%

PAUSE