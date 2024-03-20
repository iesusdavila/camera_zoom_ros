cmake_minimum_required(VERSION 3.0.2)
project(camera_zoom_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)
 
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
 
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
 
catkin_package(
  DEPENDS
    rospy
    sensor_msgs
    std_msgs
)

