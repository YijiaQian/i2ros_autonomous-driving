cmake_minimum_required(VERSION 3.0.2)
project(map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(generatePointCloud_node src/generatePointCloud.cpp)

target_link_libraries(generatePointCloud_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(generatePointCloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
