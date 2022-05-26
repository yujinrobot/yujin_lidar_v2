cmake_minimum_required(VERSION 3.0.2)
project(yrl_to_cloud)

find_package(catkin REQUIRED COMPONENTS
  laser_assembler
  rospy
)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yrl_to_cloud
#  CATKIN_DEPENDS laser_assembler rospy
#  DEPENDS system_lib
#)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
