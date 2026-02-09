cmake_minimum_required(VERSION 3.0.2)
project(sonar_camera_reconstruction_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    pybind11_catkin
    rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  DEPENDS EIGEN3
)

pybind11_add_module(cfar src/${PROJECT_NAME}/cpp/cfar.cpp)
set_target_properties(cfar PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

