<h2>OpenMMS Post-Processing Software</h2>

| IMPORTANT NOTE | 
| ---------------|
| The current OpenMMS Hardware utilizes an Applanix GNSS-INS sensor, and therefore requires the use of the Applanix POSPac software application in order to process the .T04 file collected from an OpenMMS sensor. **Currently, POSPac can ONLY be installed within a Windows OS environment.** |

<p align="center">
<img src="../images/openmms_oss.png">
</p>

<p>The following command-line-based Python3 applications are installed on a user's computer, and provide the user with a set of tools for handling all the required MMS data related tasks. The applications have been optimized for performance and speed by utilizing multi-core processing whenever possible.</p>

<p>&nbsp;&nbsp;&nbsp;<a href="./code/openmms_pcap_check.py">OpenMMS PCAP Check</a>
<br>&nbsp;&nbsp;&nbsp;<a href="./code/openmms_traj_convert.py">OpenMMS TRAJ Convert</a>
<br>&nbsp;&nbsp;&nbsp;<a href="./code/openmms_georeference.py">OpenMMS Georeference</a>
<br>&nbsp;&nbsp;&nbsp;<a href="./code/openmms_preprocess_images.py">OpenMMS Preprocess Images</a>
<br>&nbsp;&nbsp;&nbsp;<a href="./code/openmms_colorize.py">OpenMMS Colorize</a></p>

| FYI / A WORK IN PROGRESS | 
| ---------------|
| A current development effort (as of June 2020) is underway to leverage NVidia CUDA GPU processing techniques within the OpenMMS software suite for the Windows 10 OS. Future versions of the OpenMMS applications will support NVidia CUDA processing, when possible. |

<p>The following command-line-based Python3 application (also available with CUDA GPU processing) is installed on a user's computer, and provides the user with a novel approach to estimating the boresight alignment angles for the OpenMMS lidar sensor. The boresight calibration approach attempts to determine the optimal set of boresight alignment angles that minimize the RMS values for a set of best-fit planar surfaces that correspond to manually identified sections within a post-processed point cloud. A novel procedure for estimating the lever-arm offsets between the GNSS antennas and the Applanix sensor is also included. A Microsoft Excel spreadsheet template is provided, where the observations recorded by the user, while following the novel procedure, are simply entered into the spreadsheet and then all calculations are performed and the lever-arm offset values are displayed back to the user. The offset values are then entered into the Applanix sensor's firmware, via its WebUI. See the OpenMMS website for complete details.</p>
  
<p>&nbsp;&nbsp;&nbsp;<a href="./code/openmms_vlp16_calibration.py">OpenMMS VLP-16 Calibration</a>
<br>&nbsp;&nbsp;&nbsp;<a href="./code/openmms_vlp16_calibration_CUDA.py">OpenMMS VLP-16 Calibration (CUDA)</a>
<br>&nbsp;&nbsp;&nbsp;<a href="./code/openmms_lever_cal.xlsx">OpenMMS Lever-arm Calibration</a></p>

<p align="center"><br>
<img width="60%" src="../images/i_heart_lidar.png">
</p>
