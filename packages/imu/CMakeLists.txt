cmake_minimum_required(VERSION 2.8.3)
project(imu)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs # Every duckietown packages should use this.
  sensor_msgs
  geometry_msgs
)

catkin_package()

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
