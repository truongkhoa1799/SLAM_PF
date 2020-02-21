cmake_minimum_required(VERSION 2.8.3)
project(lpn_global_path_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  nav_core
  nav_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  tf
  costmap_2d
  base_local_planner
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
)

#
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(lpn_global_path_planner
        src/lpn_global_path_planner.cpp
 )
 target_link_libraries(lpn_global_path_planner
   ${catkin_LIBRARIES}
 )