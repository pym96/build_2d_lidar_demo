cmake_minimum_required(VERSION 3.0.2)
project(pcl2pcd)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

## Find PCL
find_package(PCL REQUIRED COMPONENTS common visualization)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

### Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions pcl_ros
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add your executable
add_executable(pcl_save_node src/pcl_save_node.cpp)
target_link_libraries(pcl_save_node ${catkin_LIBRARIES})

add_executable(pcl_visualization_node src/pcl_visualization.cpp)
target_link_libraries(pcl_visualization_node ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
