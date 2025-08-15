# sonar_camera_reconstruction

This repo contains the code derived from the paper "Opti-Acoustic Scene Reconstruction in Highly Turbid Underwater Environments" (2025), which presents an imaging sonar and monocular camera merging system for scene reconstruction.  
[Paper (arXiv)](https://arxiv.org/abs/2508.03408)  
  
If you are looking for a ROS2 version of this code it can be found [here](https://github.com/ivanacollg/sonar_camera_reconstruction/tree/ROS2).

![GIF](sonar_camera_reconstruction/utils/Readme.gif)

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

### Subscriber Topics:
- #### Camera image topic:  
    - Default Name: /camera/image_raw/compressed
    - Type: sensor_msgs/msg/CompressedImage
- #### Sonar topic:
    - Default Name: /sonar_oculus_node/M750d/ping
    - Type: sonar_oculus/msg/OculusPing
- #### Robot Odometry topic:
    - Default Name: /bruce/slam/localization/odom
    - Type: nav_msgs/msg/Odometry

### Publisher Topics:
- #### Segmented Image topic
    - Default Name: /sonar_camera_reconstruction/segmented_img/compressed
    - Type: sensor_msgs/msg/CompressedImage
- #### Sonar features Image topic
    - Default Name: /sonar_camera_reconstruction/feature_img/compressed
    - Type: sensor_msgs/msg/CompressedImage
- #### Point Cloud reconstruction
    - Default Name: /sonar_camera_reconstruction/cloud
    - Type: sensor_msgs/msg/PointCloud2

### Parameter Files:
#### Monocular Camera parameters
- image_width: width of the image in pixels
- image_height: height of the image in pixels
- camera_matrix/data:
  - fx - focal length in pixels along the x axis
  - fy - focal length in pixels along the y axis
  - s - skew term (normally 0 unless your camera pixels are not perfectly rectangular)
  - (cx, cy) - coordinates of the principle point (optical center) in pixels
  - Complete camera matrix:
    <pre>
    [fx,  s,  cx,  
      0, fy,  cy,  
      0,  0,   1 ]
    </pre>


- distortion_coefficients/data: by default uses the plumb bob model, can be found through camera calibration
- Ts_c - Transformation Matrix from sonar to camera frame:
  - contains a rotation matrix R, a transformation matrix T, and a 4th row to make it homogeneous (never changes)
  - R is a 3x3 matrix with each column denoting the camera's x, y, and z axis directions in relation to the sonar axes in unit vector form.
  - T is a 1x3 matrix with each row denoting the x, y, and z coordinates of the origin of the camera frame in relation to the origin of the sonar frame in meters.
  - Complete transformation matrix:
    <pre>
     [R<sub>xx</sub>, R<sub>yx</sub>, R<sub>zx</sub>, T<sub>x</sub>,
      R<sub>xy</sub>, R<sub>yy</sub>, R<sub>zy</sub>, T<sub>y</sub>,
      R<sub>xz</sub>, R<sub>yz</sub>, R<sub>zz</sub>, T<sub>z</sub>,
       0,   0,   0,  1,]
    </pre>

#### Sonar parameters
- sonarRange: The maximum detection range in meters
- thresholdHorizontal: detection strength threshold for the horizontal sonars CFAR processing, higher thresholds remove both noise as well as weaker targets in favor of fewer, stronger targets 
- verticalAperture: The vertical angular coverage in degrees
- sonar_features: True, publishe sonar feature images

- CFAR/Ntc: number of training cells
- CFAR/Ngc: number of guard cells
- CFAR/Pfa: false alarm rate
- CFAR/rank: matrix rank

#### Merge parameters
- min_area: minimum # of pixels in the image to be considered contact
- threshold_inv: True, boolean to choose whether to invert the threshold to look for dark or light areas in the image (Dark = True, Light = False)
- boundary: # of pixels in the boundary to not be included in the segmentation
- fast_performance: True, code will run faster but become slightly less precise



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
