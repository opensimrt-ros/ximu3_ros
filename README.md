# ximu3_ros

Simple tf publisher based on the Cpp Udp examples from https://github.com/xioTechnologies/x-IMU3-Software

Library `libximu3.a` was compiled from the rust source and the binary copied over. 



## TODO:

- consider adding code for generating libximu3.a as well (with a custom target)
- make sure parametrization works with multiple IMUs
- remove incorrect code for PoseStamped publisher
- add other data from sensor to publish complete ros IMU sensor message
