#!/bin/bash

# References
# http://wiki.ros.org/noetic/Installation/Ubuntu

#Terminal colors
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
blue=`tput setaf 4`
magenta=`tput setaf 5`
cyan=`tput setaf 6`
reset=`tput sgr0`

###############################
### Create Catkin Worksapce ###
###############################

ROS_VER=""

echo "${green}Preparing Catkin Workspace${reset}"
echo "What is the name of your Catkin Workspace?"
read varname
WS_NAME=${varname}_ws
echo "${yellow}Preparing to create workspace ${WS_NAME}${reset}"

mkdir -p ~/${WS_NAME}/src
cd ~/${WS_NAME}/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH
