cmake_minimum_required(VERSION 3.0.2)
project(gazebo_tb3_car_explore)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/explore_mapping.py
  scripts/record_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
