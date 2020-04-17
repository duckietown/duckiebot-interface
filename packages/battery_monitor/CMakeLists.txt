cmake_minimum_required(VERSION 2.8.3)
project(battery_monitor)


find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS duckietown_msgs rospy
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
