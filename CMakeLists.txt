cmake_minimum_required(VERSION 3.0.2)
project(cloud_to_map)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  pcl_ros
  pcl_conversions 
)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cloud_to_map
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs nav_msgs pcl_ros pcl_conversions
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(cloud_to_map_node src/cloud_to_map.cpp)
target_link_libraries(cloud_to_map_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
