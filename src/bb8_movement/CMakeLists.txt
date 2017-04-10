cmake_minimum_required(VERSION 2.8.3)
project(bb8_movement)

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
    include
    ${catkin_INCLUDE_DIRS}
    ${bb8_communications_INCLUDE_DIRS}
)

add_executable(
    bb8_movement_node src/Move.cpp src/Motor.cpp src/Servo.cpp
)

target_link_libraries(
    bb8_movement_node ${catkin_LIBRARIES} bb8_communications_node
)
