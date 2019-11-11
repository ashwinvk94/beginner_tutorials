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
source devel/setup.bash
roslaunch beginner_tutorials Week11_HW.launch
```

## Inspecting TF Frames
First run the launch file as mentioned above
Then run the below scripts to visualize the tf topics
```
cd <path to catkin workspace>
rosrun rqt_tf_tree rqt_tf_tree
```

To echo the value types, do the following:
```
rosrun tf tf_echo /world /talk
```

To produce a diagram of the broadcaster frame:
```
rosrun tf view_frames
```
## Running ROS tests

To run the tests:
```
catkin_make run_tests beginner_tutorials
```
## Recording using rosbag
In a terminal run `roslaunch beginner_tutorials Week11_HW.launch`
In another terminal run the follwing commands:
```
cd <path to catkin workspace>/src/beginner_tutorials/results
rosbag record -a -O rostopicsRecord.bag
```

## Playing using rosbag
In a new terminal run `roscore`
In another terminal run `rosrun beginner_tutorials listener`
In another terminal run the follwing commands:
```
cd <path to catkin workspace>/src/beginner_tutorials/results
rosbag play rostopicsRecord.bag
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
