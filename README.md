# realsense_multi_cam
A wrapper around librealsense cpp-multicam example to publish multiple color images and depth images, depending on how many devices are connected.

Please install librealsense according to its installation guide [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

One will have to change the hardcoded include and lib path in CMakeLists.txt to include the proper library path.
