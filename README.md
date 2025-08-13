# sonar_camera_reconstruction

This repo contains the code derived from the paper "Opti-Acoustic Scene Reconstruction in Highly Turbid Underwater Environments" (2025), which presents an imaging sonar and monocular camera merging system for scene reconstruction. 

# Dependencies
This codebase is ROS native and will require a ROS installation. It can be used without ROS, but will require some work.

    - ROS Noetic
    - Python3
    
Dependencies:
```
    sudo pip install catkin_tools scipy open3d gtsam
    sudo apt-get install ros-noetic-pybind11-catkin
```
```
    sudo apt-get -y install python3-dev python3-pip
    sudo pip install  mavproxy pymavlink cv2
    
    # install ROS dependencies
    sudo apt-get install ros-noetic-joy ros-noetic-cv-bridge ros-noetic-nav-core ros-noetic-cv-bridge ros-noetic-tf2-geometry-msgs
```

Presently, this codebase uses our sonar image system in Argonaut `https://github.com/jake3991/Argonaut.git`.

# Set Up
```
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone https://github.com/ethz-asl/libpointmatcher.git
    git clone https://github.com/ethz-asl/libnabo.git  
    git clone https://github.com/jake3991/Argonaut.git
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

# Use Guide
Input Topics:
- Camera image topic
- Sonar topic
- Robot Odometry topic 

Output Topics:
- Segmented Image topic
- Sonar features Image topic
- Point Cloud reconstruction

Parameters:
- Monocular Camera parameters
- Sonar parameters
- Merge parameters



# Citations
If you use this repo please cite the following work. 
```
@misc{collado2025,
      title={Opti-Acoustic Scene Reconstruction in Highly Turbid Underwater Environments}, 
      author={Ivana Collado-Gonzalez and John McConnell and Paul Szenher and Brendan Englot},
      year={2025},
      eprint={2508.03408},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2508.03408}, 
}
```
