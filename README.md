# camera_calib
## Quick Start
### Make transform matrix

'''
$ python3 gen_cameras_matrix.py
'''

**Note:　Global variables that need to be modified (line 10 - 19).**
1. save_path：Path name to save the transform matrix.
2. CAMERA_ID_n：ID of RealSense to be used.
3. MESUREMENT_TIME：Time to observe the marker.
4. MEASUREMENT_NUM：Number of observation times.
