# Nagatsuki

[![Build status](https://ci.appveyor.com/api/projects/status/5qbnuk7k0iagj7av?svg=true)](https://ci.appveyor.com/project/Panepo/nagatsuki)

An implementation of ORB_SLAM2 in Windows platform with webcam or Intel® RealSense™

## Thanks

* [Mur-Atal](https://github.com/raulmur)
* [ORB-SLAM](https://github.com/raulmur/ORB_SLAM)
* [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)

## FAQ

### What is this?

This is an implementation of visual SLAM algorithm using ORB_SLAM2 library in Windows platform with your webcam or Intel® RealSense™. ORB_SLAM2 is a real-time SLAM library for Monocular, Stereo and RGB-D cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale).

### SLAM?

Simultaneous localization and mapping (SLAM) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

### Augmented Reality?

Right, SLAM is the basis of the augmented reality: The device needed to know it's 3D position in the world. When an AR app starts, the system doesn’t know much about the environment. It starts processing data from various sensors like camera. To improve accuracy, the device combines data from other useful sensors like the accelerometer and the gyroscope.

### Visual SLAM

Visual SLAM is the SLAM system which process data only come from the camera, so doesn’t use odometry by accelerometers and gyroscopes. A very well working and recent algorithm is [ORB-SLAM](https://github.com/raulmur/ORB_SLAM) by [Mur-Atal](https://github.com/raulmur), Montiel and Tardós. The successor [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) adds support for stereo or depth cameras in addition to a monocular system. What’s especially great is that the algorithms are available as open source under the GPL-v3 license.

## Prerequisite

* OpenCV 3.4.3
* [ORB_SLAM24Windows](https://github.com/phdsky/ORBSLAM24Windows)
* [librealsense](https://github.com/IntelRealSense/librealsense) 2.16.3
* Visual Studio 2015
* Python 3.6

## Requirements

* Intel® RealSense™ 415/435
* Webcam

## Usage and Results

### KITTI Dataset

1. Download the dataset (grayscale images) from [http://www.cvlibs.net/datasets/kitti/eval_odometry.php](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
2. Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11.

### TUM Dataset

1. Download a sequence from [http://vision.in.tum.de/data/datasets/rgbd-dataset/download](http://vision.in.tum.de/data/datasets/rgbd-dataset/download) and uncompress it.
2. Associate RGB images and depth images using the python script [tum-associate.py](https://github.com/Panepo/Nagatsuki/blob/master/Scripts/tum-associate.py). We already provide associations for some of the sequences in Examples/RGB-D/associations/. You can generate your own associations file executing:
```
python tum-associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
```
3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

### Webcam

Grab webcam calibration and distortion parameters by yourselves, there is no common methods to get them.

* RGB Mode: only RGB mode is allowed for webcam.

### Intel® RealSense™ 415/435

First get RealSense intrin and extrin parameter using this python script [getRealsense.py](https://github.com/Panepo/Nagatsuki/blob/master/Scripts/getRealsense.py), then fill the parameter to realsense.yaml, realsense-stereo.yaml and realsense-rgbd.yaml for RGB, Stereo and RGBD mode respectively.

* RGB Mode

![result-rgb](https://github.com/Panepo/Nagatsuki/blob/master/doc/result-rgb.png)

Input source is set as RGB camera of RealSense. For an enclosing path, there might some route error.

* Stereo Mode

Input source is set as two infrared cameras of RealSense. Due to the brightness of the infrared cameras, feature extracting, matching and tracking is quite hard.

* RGBD Mode

![result-rgbd](https://github.com/Panepo/Nagatsuki/blob/master/doc/result-rgbd.png)

Input source is set as RGB camera and depth frame of RealSense. The best result within 3 modes with RealSense
