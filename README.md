# ROS Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## Overview

This repository contains the code to implement the tutorials for ROS provided on the [official website](http://wiki.ros.org/ROS/Tutorials#Beginner_Level).

## Dependencies
- [Robot Operating System (ROS Kinetic)](http://wiki.ros.org/kinetic/Installation) (middleware for robotics),


## Standard build and run using roslaunch
```
cd <path to catkin workspace>/src
git clone --recursive https://github.com/ashwinvk94/beginner_tutorials
cd ../..
catkin_make
source devel/setup.bashroslaunch beginner_tutorials Week11_HW.launch
```
## Change text using rosservice
```
rosservice call /changeText "Text has been updated"
```

## Cpplint check
Execute the following commands in a new terminal to run cpplint
```
cd  <path to repository>
cpplint $( find . -name \*.hpp -or -name \*.cpp )
```

## Cppcheck chec
Execute the following commands in a new terminal to run cppcheck
```
cd <path to repository>
cppcheck --enable=all --std=c++11 -I ../../devel/include/ -I ../../../../../../../opt/ros/kinetic/include/ -I ../../../../../../../usr/include/ --check-config --suppress=missingIncludeSystem $( find . -name *.cpp)
```
