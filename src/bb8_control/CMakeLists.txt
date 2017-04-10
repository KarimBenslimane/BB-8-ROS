cmake_minimum_required(VERSION 2.8.3)
project(bb8_control)

## Add support for C++11, supported in ROS Kinetic and newer
 add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
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
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    bb8_control_node src/sixaxis.cpp
)

target_link_libraries(
    bb8_control_node ${catkin_LIBRARIES}
)