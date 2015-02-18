cmake_minimum_required(VERSION 2.8.3)
project(rose_system_validation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  sensor_msgs
  dynamixel_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy std_msgs sensor_msgs dynamixel_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
