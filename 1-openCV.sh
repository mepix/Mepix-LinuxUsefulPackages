#!/bin/bash

# References
# https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/
# https://github.com/jetsonhacks/buildOpenCVTX2/blob/master/buildAndPackageOpenCV.sh

#Terminal colors
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
blue=`tput setaf 4`
magenta=`tput setaf 5`
cyan=`tput setaf 6`
reset=`tput sgr0`

#Files
INSTALL_DIR=${HOME}/Install
OPENCV_VER=3.4.1

echo "${blue}Preparing to build OpenCV ${OPENCV_VER} in ${INSTALL_DIR}${reset}"

##################################
### Install Dependencies Tools ###
##################################

echo "${green}Installing OpenCV Dependencies${reset}"
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install build-essential cmake unzip pkg-config -y
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev -y
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev -y
sudo apt-get install libxvidcore-dev libx264-dev -y
sudo apt-get install libgtk-3-dev -y
sudo apt-get install libatlas-base-dev gfortran -y
sudo apt-get install python3-dev -y

sudo apt-get install -y \
    cmake \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libeigen3-dev \
    libglew-dev \
    libgtk2.0-dev \
    libgtk-3-dev \
    libjasper-dev \
    libjpeg-dev \
    libpng12-dev \
    libpostproc-dev \
    libswscale-dev \
    libtbb-dev \
    libtiff5-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    qt5-default \
    zlib1g-dev \
    pkg-config

####################################
### Download Library Source Code ###
####################################

echo "${yellow}Checking for INSTALL directory${reset}"
if [ ! -d "$INSTALL_DIR" ]; then
  # Control will enter here if $DIRECTORY doesn't exist.
  echo "${cyan}Creating directory: ${INSTALL_DIR}"
  mkdir $INSTALL_DIR
fi

cd $INSTALL_DIR
echo "${green}Downloading OpenCV Source${reset}"
wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VER}.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VER}.zip

unzip opencv.zip
unzip opencv_contrib.zip

mv opencv-${OPENCV_VER} opencv
mv opencv_contrib-${OPENCV_VER} opencv_contrib

##############################
### Provide Python Support ###
##############################

echo "${green}Adding Python3 Support${reset}"
echo "${red}PYTHON NOT CURRENTLY SUPPORTED${reset}"
#TODO: fix python support
# wget https://bootstrap.pypa.io/get-pip.py
# sudo python3 get-pip.py
#
# sudo pip install virtualenv virtualenvwrapper
# sudo rm -rf ~/get-pip.py ~/.cache/pip
#
# echo -e "\n# virtualenv and virtualenvwrapper" >> ~/.bashrc
# echo "export WORKON_HOME=$HOME/.virtualenvs" >> ~/.bashrc
# echo "export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3" >> ~/.bashrc
# echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc
#
# source ~/.bashrc
#
# echo $WORKON_HOME
# echo $VIRTUALENVWRAPPER_PYTHON

# Create OpenCV Virtual Environment
# mkvirtualenv cv -p python3
# workon cv
#
# sudo pip install numpy
#

####################
### Build OpenCV ###
####################

echo "${green}BUILD${reset}"

cd $INSTALL_DIR/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
  -D WITH_CUDA=ON \
  -D CUDA_ARCH_PTX="" \
  -D ENABLE_FAST_MATH=ON \
  -D CUDA_FAST_MATH=ON \
  -D WITH_CUBLAS=ON \
  -D WITH_LIBV4L=ON \
  -D WITH_GSTREAMER=ON \
  -D WITH_GSTREAMER_0_10=OFF \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D INSTALL_C_EXAMPLES=ON \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D OPENCV_EXTRA_MODULES_PATH=$INSTALL_DIR/opencv_contrib/modules \
  -D WITH_QT=ON \
  -D WITH_OPENGL=ON \
  -D OPENCV_ENABLE_NONFREE=ON \
	-D BUILD_EXAMPLES=ON ..

#  -D CUDA_ARCH_BIN=${ARCH_BIN} \
# -D PYTHON_EXECUTABLE=~/.virtualenvs/cv/bin/python \


echo "${green}MAKE${reset}"
make -j4

sudo make install
sudo ldconfig

####################
### Test Install ###
####################

echo "${green}Testing OpenCV Install${reset}"
echo "${magenta}OpenCV Version:{cyan}"
pkg-config --modversion opencv
echo "${magenta}OpenCV can be found in the following directories:${cyan}"
pkg-config --cflags opencv
echo "${magenta}Included OpenCV libraries:${cyan}"
pkg-config --libs opencv
echo "${reset}"
