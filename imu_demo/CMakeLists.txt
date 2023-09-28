cmake_minimum_required(VERSION 3.0.2)
project(imu_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Setting Cmake epxort command on
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)
find_package(serial REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/imu_demo.cpp)
add_executable(${PROJECT_NAME}_sub_node src/imu_sub.cpp)
add_executable(${PROJECT_NAME}_quaternion_node src/imu_quaternion_output.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${serial_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_sub_node ${catkin_LIBRARIES} ${serial_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_quaternion_node ${catkin_LIBRARIES} ${serial_LIBRARIES})

