#!/bin/bash

# References
# https://github.com/Kinovarobotics/kinova-ros/blob/melodic-devel/kinova_moveit/MoveIt.md
#Terminal colors
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
blue=`tput setaf 4`
magenta=`tput setaf 5`
cyan=`tput setaf 6`
reset=`tput sgr0`

ROS_VER="melodic"
sudo apt-get install ros-${ROS_VER}-moveit -y
sudo apt-get install ros-${ROS_VER}-trac-ik -y
cd ~/kinova_ws
catkin_make
