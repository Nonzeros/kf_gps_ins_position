# Install script for directory: /home/ouyy/MyRosGuideCar/src/rosmsgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ouyy/MyRosGuideCar/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gtec_msgs/msg-ros1" TYPE FILE FILES
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg"
    "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gtec_msgs/cmake" TYPE FILE FILES "/home/ouyy/MyRosGuideCar/build/rosmsgs/catkin_generated/installspace/gtec_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ouyy/MyRosGuideCar/devel/include/gtec_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ouyy/MyRosGuideCar/devel/share/roseus/ros/gtec_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ouyy/MyRosGuideCar/devel/share/common-lisp/ros/gtec_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ouyy/MyRosGuideCar/devel/share/gennodejs/ros/gtec_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ouyy/MyRosGuideCar/devel/lib/python3/dist-packages/gtec_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ouyy/MyRosGuideCar/devel/lib/python3/dist-packages/gtec_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ouyy/MyRosGuideCar/build/rosmsgs/catkin_generated/installspace/gtec_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gtec_msgs/cmake" TYPE FILE FILES "/home/ouyy/MyRosGuideCar/build/rosmsgs/catkin_generated/installspace/gtec_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gtec_msgs/cmake" TYPE FILE FILES
    "/home/ouyy/MyRosGuideCar/build/rosmsgs/catkin_generated/installspace/gtec_msgsConfig.cmake"
    "/home/ouyy/MyRosGuideCar/build/rosmsgs/catkin_generated/installspace/gtec_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gtec_msgs" TYPE FILE FILES "/home/ouyy/MyRosGuideCar/src/rosmsgs/package.xml")
endif()

