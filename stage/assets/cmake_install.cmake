# Install script for directory: /home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/opt/ros/melodic")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RELEASE")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stage/assets" TYPE FILE FILES
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/blue.png"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/death.png"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/green.png"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/logo.png"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/mains.png"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/mainspower.png"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/question_mark.png"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/red.png"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/stagelogo.png"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/stall.png"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/rgb.txt"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stage" TYPE FILE FILES "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/assets/rgb.txt")
endif()

