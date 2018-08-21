#Maplab

https://github.com/ethz-asl/kalibr


https://github.com/ethz-asl/maplab
https://github.com/ethz-asl/maplab/wiki/Installation-Ubuntu


##Create a catkin workspace

To create a workspace, run:

export ROS_VERSION=kinetic #(Ubuntu 16.04: kinetic, Ubuntu 14.04: indigo)
export CATKIN_WS=~/Workspace/maplab_ws
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src

##Cloning over HTTPS (no github account needed)

git clone https://github.com/ethz-asl/maplab.git --recursive
git clone https://github.com/ethz-asl/maplab_dependencies --recursive

##Setting up the linter

This setups a linter which checks if the code conforms to our style guide during commits. These steps are only necessary if you plan on contributing to maplab.

cd $CATKIN_WS/src/maplab
./tools/linter/init-git-hooks.py
*NOTE: DID NOT DO THIS STEP*

##Building maplab

cd $CATKIN_WS
catkin build maplab
