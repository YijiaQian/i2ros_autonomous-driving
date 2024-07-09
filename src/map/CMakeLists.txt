cmake_minimum_required(VERSION 3.0.2)
project(map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  tf2_ros
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(generatePointCloud src/generatePointCloud.cpp)
target_link_libraries(generatePointCloud ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pcl_common pcl_io)

install(TARGETS generatePointCloud
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
