# Install script for directory: /home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/src/gps_simulator

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gps_simulator/msg" TYPE FILE FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/src/gps_simulator/msg/gps_measurement.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gps_simulator/cmake" TYPE FILE FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/build/gps_simulator/catkin_generated/installspace/gps_simulator-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/devel/include/gps_simulator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/devel/share/roseus/ros/gps_simulator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/devel/share/common-lisp/ros/gps_simulator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/devel/share/gennodejs/ros/gps_simulator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/devel/lib/python2.7/dist-packages/gps_simulator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/devel/lib/python2.7/dist-packages/gps_simulator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/build/gps_simulator/catkin_generated/installspace/gps_simulator.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gps_simulator/cmake" TYPE FILE FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/build/gps_simulator/catkin_generated/installspace/gps_simulator-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gps_simulator/cmake" TYPE FILE FILES
    "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/build/gps_simulator/catkin_generated/installspace/gps_simulatorConfig.cmake"
    "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/build/gps_simulator/catkin_generated/installspace/gps_simulatorConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gps_simulator" TYPE FILE FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/src/gps_simulator/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gps_simulator/gps_simulator" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gps_simulator/gps_simulator")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gps_simulator/gps_simulator"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gps_simulator" TYPE EXECUTABLE FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/devel/lib/gps_simulator/gps_simulator")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gps_simulator/gps_simulator" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gps_simulator/gps_simulator")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gps_simulator/gps_simulator"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gps_simulator/gps_simulator")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gps_simulator" TYPE DIRECTORY FILES "/home/william/Documents/Spaceryde_Design_Challenge/Spaceryde_Design_Challenge/catkin_ws/src/gps_simulator/include/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hpp$")
endif()

