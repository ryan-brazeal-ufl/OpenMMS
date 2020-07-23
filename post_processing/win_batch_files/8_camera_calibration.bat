@ECHO OFF

set max_images=500

python C:\OpenMMS\code\openmms_camera_calibration.py %cd% %max_images%

PAUSE