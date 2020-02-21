# Install script for directory: /home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stage/worlds" TYPE FILE FILES
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/amcl-sonar.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/autolab.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/camera.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/everything.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/lsp_test.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/mbicp.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/nd.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/roomba.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/simple.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/test.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/uoa_robotics_lab.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/vfh.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/wavefront-remote.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/wavefront.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/wifi.cfg"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/SFU.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/autolab.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/camera.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/everything.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/fasr.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/fasr2.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/fasr_plan.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/large.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/lsp_test.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/mbicp.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/pioneer_flocking.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/pioneer_walle.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/roomba.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/sensor_noise_demo.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/sensor_noise_module_demo.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/simple.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/uoa_robotics_lab.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/wifi.world"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/beacons.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/chatterbox.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/hokuyo.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/irobot.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/map.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/objects.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/pantilt.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/pioneer.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/sick.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/ubot.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/uoa_robotics_lab_models.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/walle.inc"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/cfggen.sh"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/test.sh"
    "/home/khoa1799/catkin_ws/src/SLAM/stage/Stage/worlds/worldgen.sh"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/khoa1799/catkin_ws/src/SLAM/stage/worlds/benchmark/cmake_install.cmake")
  include("/home/khoa1799/catkin_ws/src/SLAM/stage/worlds/bitmaps/cmake_install.cmake")
  include("/home/khoa1799/catkin_ws/src/SLAM/stage/worlds/wifi/cmake_install.cmake")

endif()

