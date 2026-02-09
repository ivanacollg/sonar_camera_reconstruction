cmake_minimum_required(VERSION 3.0.2)
project(sonar_oculus)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
)

## 2. Declare messages
add_message_files(
  FILES
  OculusPing.msg
  OculusFire.msg
)

## 3. Generate messages
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

## 4. Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    sensor_msgs
)