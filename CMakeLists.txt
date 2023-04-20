cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_merger)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp pcl_ros sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_merger_node src/pointcloud_merger_core.cpp src/pointcloud_merger_node.cpp)
target_link_libraries(pointcloud_merger_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS pointcloud_merger_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)