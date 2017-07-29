but_calibration_camera_velodyne
===============================

ROS packages for extrinsic calibration Velodyne 3D LIDARs(vlp16) with ZED stereo camera.
The package follows the methods proposed by:

* [1] *Velas, Spanel, Materna, Herout: Calibration of RGB Camera with Velodyne LiDAR*

* [2] Jingfeng Sui, Shuo Wang, Extrinsic Calibration of Camera and 3D Laser Sensor System. 

Requirements
------------

* PCl 1.7 and OpenCV 3.x library

Usage
-----

- `roslaunch but_calibration_camera_velodyne calibration_coarse.launch`
    - command will run the coarse calibration using 3D marker (described in the [1])
    - if the 3D marker detection fails, the attempt will be repeated after 5s
    - upon the cuccess the node ends and prints the 6 Degrees of Freedom of the Velodyne related to the camera

- `roslaunch but_calibration_camera_velodyne calibration_fine.launch`
    - will runs the calibration with the refinement. This process will take a longer time (approx. 1 minute)

- `roslaunch but_calibration_camera_velodyne coloring.launch`
    - launches the publisher of colored pointclouds created by the camera -- Velodyne fusion
    - the calibration parameters (6DoF) computed by the `calibration` process must be set in configuration file `conf/coloring.yaml` (see below)

Configuration:
--------------

Configuration *.yaml files are stored in the direcotry `conf/`:

- `conf/calibration.yaml` contains folowing parameters:
    - `camera_frame_topic`: name of the topic, where camera RGB image is published
    - `camera_info_topic`: topic of `CameraInfo` type
    - `velodyne_topic`: topic, where the Velodyne pointcloud is published
    - `marker`: physical size of the 3D marker 
        - `circles_distance`: distance between the circles centres (see [1])
        - `circles_radius`: radius of the circles in marker (also see [1])
- in `conf/calibration.yaml` you can specify:
    - `6DoF`: vector of the calibration parameters computed by the calibration process
    - `velodyne_color_topic`: name of the topic, where the colored pointcloud will be published

Sensors Setup:
--------------
![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/sensor.JPG)
![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/experimentSmall.JPG)
![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/frames.png)


Calibration Target:
-------------------
![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/calibrationBoardFrontSmall.JPG)

Original Images:
------------------
![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/frame_rgb.JPG)

2D edges:
------------------

![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/ImgEdge.JPG)
![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/IDTE2.JPG)
![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/HoughCircle.jpg)

Original Pointcloud:
------------------
![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/frontview.jpg)
![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/top_view.jpg)


3D edges:
------------------
![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/edgespoint_s.png)


3d edges point reproject to image:
-------------
![](https://github.com/suijingfeng/but_velodyne/blob/master/but_calibration_camera_velodyne/doc/cccOri/fig/fine_calib_proj.jpg)


