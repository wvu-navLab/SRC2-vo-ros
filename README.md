# wvu_vo_ros
Code for wvu_vo and ROS wrapper for wvu_vo

**The submodule was merged into this repository, so the submodule commands are no longer needed.** 

~~To update submodule after pulling, run git submodule update --init --recursive~~ 

## Requirements (wvu_vo_ros)
This code requires ROS Kinetic (with `ros-(version)-ros-base`, `ros-(version)-cv-bridge`, ~~`ros-(version)-gazebo-msgs`~~, `ros-(version)-image-transport`, `ros-(version)-tf`), OpenCV 3.2.0 or greater, and C++11. ~~To install OpenCV, an install script is provided [here](https://github.com/wvu-irl/wvu_vo/blob/master/scripts/install_opencv.sh).~~ OpenCV no longer needs installed separately as OpenCV 3.2.0 is installed by default along with the ROS packages. To check OpenCV version, run `pkg-config --modversion opencv` to check if version 3.2.0 is installed. 

## Requirements (wvu_vo)
This code requires C++11 and OpenCV 3.2.0 or greater. To install OpenCV, an [install script](https://github.com/wvu-irl/wvu_vo/blob/master/scripts/install_opencv.sh) is provided in the scripts folder. NOTE: If using this library with the [ROS wrapper](https://github.com/wvu-irl/wvu_vo_ros), OpenCV likely does not need to be installed separately.

## Note
Initial commit copied from **https://github.com/wvu-irl/wvu_vo_ros** and **https://github.com/wvu-irl/wvu_vo**
