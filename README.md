# ROS Tutorials

[![Build Status](https://travis-ci.org/ashwinvk94/beginner_tutorials.svg?branch=master)](https://travis-ci.org/ashwinvk94/beginner_tutorials)
---

## Overview

This repository contains the code to implement the tutorials for ROS provided on the [official website](http://wiki.ros.org/ROS/Tutorials#Beginner_Level).

## Dependencies
- [Robot Operating System (ROS Kinetic)](http://wiki.ros.org/kinetic/Installation) (middleware for robotics),


## Standard build and run via command-line
```
cd <path to catkin workspace>/src
git clone --recursive https://github.com/ashwinvk94/beginner_tutorials
cd ../..
catkin_make
source devel/setup.bash
rosrun beginner_tutorials talker
rosrun beginner_tutorials listener
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
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp )
```
