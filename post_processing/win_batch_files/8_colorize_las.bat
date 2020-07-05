@ECHO OFF
setlocal EnableExtensions EnableDelayedExpansion

set las_file=..\..\filtered.las

set color_balance=1

set do_dist_search_after=False


python C:\OpenMMS\code\openmms_colorize.py %cd% %las_file% %color_balance% %do_dist_search_after%


PAUSE