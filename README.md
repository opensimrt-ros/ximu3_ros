# ximu3_ros

Simple tf publisher based on the Cpp Udp examples from https://github.com/xioTechnologies/x-IMU3-Software

Library `libximu3.a` was compiled from the rust source and the binary copied over. 

To compile a newer version of the library you may use https://github.com/opensimrt-ros/docker-ximu3-library-builder

# Usage:

Compile package with `catkin_make`, source the catkin\_workspace and roslaunch it:

	roslaunch ximu3_ros ximu.launch

## TODO:

- make sure parametrization works with multiple IMUs
- add other data from sensor to publish complete ros IMU sensor message
