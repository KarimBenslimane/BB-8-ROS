cmake_minimum_required(VERSION 2.8.3)
project(bb8_sensor)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    bb8_communications REQUIRED
)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES bb8_communications
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    ${bb8_communications_INCLUDE_DIRS}
)
