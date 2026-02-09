# sonar_camera_reconstruction

This repo contains the code derived from the paper **"Opti-Acoustic Scene Reconstruction in Highly Turbid Underwater Environments" (2025)**, which presents an imaging sonar and monocular camera merging system for scene reconstruction.  
[Paper IEEEXplore](https://ieeexplore.ieee.org/document/11247733) | [Paper (arXiv)](https://arxiv.org/abs/2508.03408)  
  
You are viewing the ROS 1 version of this code, if you are looking for a ROS2 version of this code it can be found [here](https://github.com/ivanacollg/sonar_camera_reconstruction/tree/ROS2).

![GIF](./utils/Readme.gif)

# Dependencies
This codebase is ROS native and will require a ROS installation. It can be used without ROS, but will require some work.

    - ROS Noetic
    - Python3
    
Dependencies:
```
    sudo pip install catkin_tools scipy open3d
    sudo apt-get install ros-noetic-pybind11-catkin
```

# Set Up
```
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone https://github.com/ivanacollg/sonar_camera_reconstruction.git
    cd ..
    catkin build
    source devel/setup.bash
```

# Running Code
```
    roslaunch sonar_camera_reconstruction merge.launch
```
Download [sample data](https://drive.google.com/file/d/1WK9nXKLUET0hseZJesYIGAZKkg8h-aQo/view?usp=sharing)
```
    rosbag play sample.bag --clock
```

# Data sets

This [dataset folder](https://drive.google.com/drive/folders/1P7bbY_ikMIOyv38ZTB3Xp3M5JepqIdYG?usp=sharing) constains all the data used for the paper "Opti-Acoustic Scene Reconstruction in Highly Turbid Underwater Environments" (2025), which presents an imaging sonar and monocular camera merging system for scene reconstruction ([Paper (arXiv)](https://arxiv.org/abs/2508.03408)). 

Each folder contains data from each scenario shown in the paper: 
- tank_piers
- tank_sea_wall
- marina_pier
- marina_sea_wall

Additionally, tank scenarious include emulated turbidity examples types 5C, 7C, and 9C. STL files of ground truth structes are likewise included. 

Each folder contains original ROS1 .bag data and converted data to ROS2 folder. 

### Running code for different scenarious
Different scenarious contain slightly different parameters for sonar range, monocular camera calibration, etc. 

Change the `environment` argument in the `merge.launch file` to launch different parameters:
- marina_pier (default)
- marina_seawall
- tank (for all tank tests)

Example:
```
    roslaunch sonar_camera_reconstruction merge.launch environment:=tank
```

# Citations
If you use this repo or any of the data provided please cite the following work:
```
@INPROCEEDINGS{11247733,
  author={Collado-Gonzalez, Ivana and McConnell, John and Szenher, Paul and Englot, Brendan},
  booktitle={2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Opti-Acoustic Scene Reconstruction in Highly Turbid Underwater Environments}, 
  year={2025},
  volume={},
  number={},
  pages={1282-1289},
  keywords={Training;Visualization;Sonar;Turbidity;Object segmentation;Reconstruction algorithms;Sonar navigation;Real-time systems;Reproducibility of results;Image reconstruction},
  doi={10.1109/IROS60139.2025.11247733}}
```

# Documentation
This repo contains three packages:
### sonar_camer_merge
This package takes in sensor information (sonars, camera and odometry), performs sensor fussion and outputs a pointcloud. 
### sonar_oculus
This package declares the oculus_sonar msg type used in this work. 
