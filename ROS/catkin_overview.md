#Catkin Introduction
##Initialization
To create a workspace:
1. `mkdir -p ~/catkin_ws/src`
1. `cd ~/catkin_ws/src`
1. `catkin_init_workspace`
More informaiton can be found on the [official ROS wiki](http://wiki.ros.org/catkin/conceptual_overview)

A recommended naming convention can be found in [the ROS Documentation](http://www.ros.org/reps/rep-0128.html).

##Folder Structure
`catkin_ws`

-> `build` space for C++ packages

-> `devel` contains `setup.bash`

-> `src`

##Adding a Package to a workspace
1. Clone the package

    `cd ~/catkin_ws/src`

    `git clone [PACKAGE URL]`

1. Build the package

    `cd ~/catkin_ws`

    `catkin_make`

1. Install any missing dependencies

    `sudo apt-get install [MISSING]`

##Creating a Catkin Package
Run: `catkin_create_pkg <your_package_name> [dependency1 dependency2 …]`

Directory Structure:
- scripts (python executables)
- src (C++ source files)
- msg (for custom message definitions)
- srv (for service message definitions)
- include -> headers/libraries that are needed as dependencies
- config -> configuration files
- launch -> provide a more automated way of starting nodes
