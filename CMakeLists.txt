cmake_minimum_required(VERSION 2.8.3)
project(realsense_multi_cam)

find_package(catkin REQUIRED cv_bridge image_transport sensor_msgs std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realsense_multi_cam
  CATKIN_DEPENDS cv_bridge image_transport sensor_msgs std_msgs
#  DEPENDS system_lib
)

if(CMAKE_COMPILER_IS_GNUCC)
    ADD_DEFINITIONS ( -std=c++11 )
endif(CMAKE_COMPILER_IS_GNUCC)

# include_directories(include)
include_directories(/home/eagle/libraries/librealsense/include)
link_directories(/home/eagle/libraries/librealsense/lib)

## Declare a C++ executable
add_executable(realsense_multi_cam_node src/realsense_multi_cam.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(realsense_multi_cam_node ${catkin_LIBRARIES} realsense)

