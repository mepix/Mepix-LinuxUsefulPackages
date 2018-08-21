#ORB-SLAM 2

##Installation
ORB-SLAM 2 Install

1.) Install Pangolin

If OpenGL is not already installed:
- (deb) `sudo apt-get install libglew-dev`
- (mac) `sudo port install glew`

Then build:
````
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
````
Next `make/install` to place library in `/usr/local/lib/`
````
make
sudo make install
````
##Example Commands
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt

./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml ~/Workspace/SLAM/datasets/MH_02_easy/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/MH02.txt

./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt

./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml ~/Workspace/SLAM/datasets/MH_02_easy/mav0/cam0/data ~/Workspace/SLAM/datasets/MH_02_easy/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/MH02.txt


##More Example Commands
````
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER

./mono_kitti ~/Install/ORB_SLAM2

./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER

./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM2.yaml ~/workspace/SLAM/datasets/rgbd_dataset_freiburg2_pioneer_slam/

python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt


./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE

./Examples/RGB-D/rgbd_tum Vocabulary/txt.ORBvoc Examples/RGB-D/TUM2.yaml ~/workspace/SLAM/datasets/rgbd_dataset_freiburg2_pioneer_slam/ ~/workspace/SLAM/datasets/rgbd_dataset_freiburg2_pioneer_slam/associations.txt


./Examples/RGB-D/rgbd_tum Vocabulary/txt.ORBvoc Examples/RGB-D/TUM2.yaml ~/workspace/SLAM/datasets/rgbd_dataset_freiburg2_pioneer_slam/ ~/workspace/SLAM/datasets/rgbd_dataset_freiburg2_pioneer_slam/associations.txt

./Examples/RGB-D/rgbd_tum Vocabulary/txt.ORBvoc Examples/RGB-D/TUM2.yaml ~/workspace/SLAM/datasets/rgbd_dataset_freiburg2_pioneer_slam/ Examples/RGB-D/associations/mpc1.txt
