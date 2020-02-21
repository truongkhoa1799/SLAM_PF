cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_adaption)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package()


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
 add_executable(turtlebot_adaption src/turtlebot_adaption.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(turtlebot_adaption
   ${catkin_LIBRARIES}
 )
