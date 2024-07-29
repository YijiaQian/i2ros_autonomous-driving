cmake_minimum_required(VERSION 3.0.2)
project(map_expander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_expander
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/map_expander_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
