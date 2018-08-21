#Point Cloud Library
##Overview
> The Point Cloud Library (PCL) is an open-source library of algorithms for point cloud processing tasks and 3D geometry processing, such as occur in three-dimensional computer vision. The library contains algorithms for feature estimation, surface reconstruction, 3D registration[4], model fitting, and segmentation. It is written in C++ and released under the BSD license.

(Source: [Wikipedia](https://en.wikipedia.org/wiki/Point_Cloud_Library))

##Install
Make sure the required dependencies are present:
- Boost
- Eigen
- FLANN
- VTK

Release candidates can be found [on Github](https://github.com/PointCloudLibrary/pcl/releases).  Additional instructions can be found [here](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php).

Replace 1.7.2 with the current version and run the following code:
````
cd pcl-pcl-1.7.2 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make -j2 install
````
