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

#####################################
### Select ROS Version to Install ###
#####################################

ROS_VER=""

PS3='Please Select ROS Version to Install: '
options=("Kinetic" "Lunar" "Melodic" "Noetic" "Quit")
select opt in "${options[@]}"
do
    case $opt in
        "Kinetic")
            echo "you chose choice $REPLY which is $opt"
            ROS_VER=kinetic
            break
            ;;
        "Lunar")
            echo "you chose choice $REPLY which is $opt"
            ROS_VER=lunar
            break
            ;;
        "Melodic")
            echo "you chose choice $REPLY which is $opt"
            ROS_VER=melodic
            break
            ;;
        "Noetic")
            echo "you chose choice $REPLY which is $opt"
            ROS_VER=noetic
            break
            ;;
        "Quit")
            echo "${red}Terminating Install${reset}"
            exit
            ;;
        *) echo "invalid option $REPLY";;
    esac
done

echo "${blue}Preparing to Install ROS ${ROS_VER}${reset}"

##################################
### Install Dependencies Tools ###
##################################

echo "${green}Installing ROS Dependencies${reset}"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update

###################
### Install ROS ###
###################

echo "${green}Installing full Desktop Version${reset}"
sudo apt install ros-${ROS_VER}-desktop-full -y

#############################
### Configure Bashrc File ###
#############################

echo "${green}Adding Source Path to Bashrc${reset}"
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

###################################
### Handle Package Dependencies ###
###################################

echo "${green}Installing ROS Package Manger 7 Dependency Tools${reset}"
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install python3-rosdep -y

# For Versions before Noetic (Ubuntu 18--)
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
sudo apt install python-rosdep -y

sudo rosdep init
rosdep update

####################
### Test Install ###
####################

echo "${green}Testing ROS Install${reset}"
roswtf
