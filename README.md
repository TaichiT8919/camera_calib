# camera_calib
## Quick Start
### 1.Realsense setup
See https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

### 2.Make transform matrix

```
$ python3 gen_cameras_matrix.py
```

**Note:　Global variables that need to be modified (line 10 - 19).**

a) save_path：Path name to save the transform matrix.

b) CAMERA_ID_n：ID of RealSense to be used.

c) MESUREMENT_TIME：Time to observe the marker.

d) MEASUREMENT_NUM：Number of observation times.


### 3.Display by rviz

```
$ roslaunch t_camera cameras_launch.launch
```

**Note:　Variables that need to be modified (line 10 - 19).**

./src/realsense_transform.cpp(line 101).

a) input_csv_file_path_12：Path of the transform matrix file (.csv).

./launch/caeras_launch.launch(line 2,3)

a) camera1_id/camera2_id：ID of RealSense to be used.
