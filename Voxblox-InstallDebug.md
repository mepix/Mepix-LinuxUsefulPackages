#Voxblox
##Overview
From [github](https://github.com/ethz-asl/voxblox):
>Voxblox is a volumetric mapping library based mainly on Truncated Signed Distance Fields (TSDFs). It varies from other SDF libraries in the following ways:
> - CPU-only, can be run single-threaded or multi-threaded for some integrators
>- Support for multiple different layer types (containing different types of voxels)
>- Serialization using protobufs
>- Different ways of handling weighting during merging
>- Different ways of inserting pose information about scans
>- Tight ROS integration (in voxblox_ros package)
>- Easily extensible with whatever integrators you want
>- Features an implementation of building Euclidean Signed Distance Fields (ESDFs, EDTs) directly from TSDFs.

##Commands
To launch the library, call: `roslaunch voxblox_ros <launchscript>.launch` where `<launchscript>` is the name of the launch file which can be found in `~/catkin_ws/src/voxblox/voxblox_ros/launch`

##Installation Errors
###Lack of include guards for pybind11:
Symptoms from catkin:
````
-- pybind11 v2.2.dev0
CMake Error at voxblox/tools/pybind11/CMakeLists.txt:88 (add_library):
  add_library cannot create target "pybind11" because another target with the
  same name already exists.  The existing target is an interface library
  created in source directory
  "/home/merrick/catkin_ws/src/voxblox/tools/pybind11".  See documentation
  for policy CMP0002 for more details.
````

````
CMake Error at voxblox/tools/pybind11/CMakeLists.txt:95 (add_library):
  add_library cannot create target "module" because another target with the
  same name already exists.  The existing target is an interface library
  created in source directory
  "/home/merrick/catkin_ws/src/voxblox/tools/pybind11".  See documentation
  for policy CMP0002 for more details.
````
````
CMake Error at voxblox/tools/pybind11/CMakeLists.txt:104 (add_library):
  add_library cannot create target "embed" because another target with the
  same name already exists.  The existing target is an interface library
  created in source directory
  "/home/merrick/catkin_ws/src/voxblox/tools/pybind11".  See documentation
  for policy CMP0002 for more details.
````
The solution was to add include guards, as suggested by [this stackoverflow post](https://stackoverflow.com/questions/8439631/cmake-multiple-subprojects-using-the-same-static-library)

###Catkin Flags:
Catkin would try to compile with two many cores... causing collisions.  It is recommended that catkin is called with `-j4 -l4` on my laptop.  Note that it is useful when bebugging to use the flags `-j1 -l1`, but it is much slower.  Further discussion can be found on github about the default configuration of [catkin](https://github.com/catkin/catkin_tools/issues/84).

###OPENCV Linking Problems:
OpenCV libraries could not be detected by the installer.  I had to update the CMAKE files as described [here](https://answers.ros.org/question/56686/opencv-cmake-error/).
This error with OpenCV3.3 has been patched in [other code from ETH Zurich](https://github.com/ethz-asl/grid_map/issues/141), but not voxblox.
