<h2>OpenMMS Post-Processing Software</h2>

| IMPORTANT NOTES | 
| ---------------|
| The current OpenMMS Hardware utilizes an Applanix GNSS-INS sensor, and therefore requires the use of the Applanix POSPac software application in order to process the .T04 file collected from an OpenMMS sensor. **Currently, POSPac can ONLY be installed within a Windows OS environment.** |
| A current development effort is underway (started June 2020), to leverage NVidia CUDA GPU processing techniques within the OpenMMS software suite for the Windows 10 OS. Future versions of the OpenMMS applications will support NVidia CUDA processing, when possible. |

<p align="center">
<img src="../images/openmms_oss.png">
</p>

<p>The following command-line-based Python3 applications are installed on a user's computer, and provide the user with a set of tools for handling all the required MMS data related tasks. The applications have been optimized for performance and speed by utilizing multi-core processing whenever possible.</p>

  
| OpenMMS Application | Windows Batch Trigger | Mac/Linux Bash Trigger |
| --------------------|-----------------------|------------------------|
| <a href="./code/openmms_pcap_check.py">OpenMMS PCAP Check.py</a> | 1 | 2 |
| <a href="./code/openmms_traj_convert.py">OpenMMS TRAJ Convert.py</a> | 1 | 2 |
| <a href="./code/openmms_georeference.py">OpenMMS Georeference.py</a> | 1 | 2 |
| <a href="./code/openmms_preprocess_images.py">OpenMMS Preprocess Images.py</a> | 1 | 2 |
| <a href="./code/openmms_colorize.py">OpenMMS Colorize.py</a> | 1 | 2 |

<h3>Sensor Calibration Applications</h3>

<p>The following command-line-based Python3 application is installed on a user's computer, and provides the user with a novel approach to estimating the boresight alignment angles for the OpenMMS lidar sensor. The boresight calibration approach attempts to determine the optimal set of boresight alignment angles that minimize the RMS values for a set of best-fit planar surfaces that correspond to manually identified planar sections within a post-processed point cloud. A novel procedure for precisely estimating the lever-arm offsets between the GNSS antennas and the Applanix sensor, via a total station/theodolite intersection survey, is also included. A Microsoft Excel spreadsheet template is provided, where the survey observations are simply entered into the spreadsheet and automatically all the calculations are performed and the lever-arm offset values are estimated. The new lever-arm offset values then need to be entered into the Applanix sensor's firmware, via its WebUI. See the OpenMMS website for complete details.</p>

| OpenMMS Application | Windows Batch Trigger | Mac/Linux Bash Trigger | 
| --------------------|-----------------------|----------------------- |
| <a href="./code/openmms_vlp16_calibration.py">OpenMMS VLP-16 Calibration.py</a> | 1 | 2 |
| <a href="./code/openmms_vlp16_calibration_CUDA.py">OpenMMS VLP-16 Calibration (CUDA).py</a> | 1 | 2 |
| <a href="./code/openmms_lever_cal.xlsx">OpenMMS Lever-arm Calibration.xlsx</a> | 1 | 2 |

<p align="center"><br>
<img width="60%" src="../images/i_heart_lidar.png">
</p>
