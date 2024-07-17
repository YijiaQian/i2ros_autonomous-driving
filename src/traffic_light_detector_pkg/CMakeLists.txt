cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_detector_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TrafficLightState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin specific configuration
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs cv_bridge std_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(traffic_light_detector_node src/traffic_light_detector_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(traffic_light_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(traffic_light_detector_node
  ${catkin_LIBRARIES}
)
