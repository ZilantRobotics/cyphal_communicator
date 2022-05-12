cmake_minimum_required(VERSION 3.0.2)
project(cyphal_communicator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
)

catkin_package(
 CATKIN_DEPENDS std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
