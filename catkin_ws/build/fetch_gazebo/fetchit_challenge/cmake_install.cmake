# Install script for directory: /home/jack/SensorsAndControlA1/catkin_ws/src/fetch_gazebo/fetchit_challenge

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jack/SensorsAndControlA1/catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/action" TYPE FILE FILES "/home/jack/SensorsAndControlA1/catkin_ws/src/fetch_gazebo/fetchit_challenge/action/SchunkMachine.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/msg" TYPE FILE FILES
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/action" TYPE FILE FILES "/home/jack/SensorsAndControlA1/catkin_ws/src/fetch_gazebo/fetchit_challenge/action/SickCamera.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/msg" TYPE FILE FILES
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraAction.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraGoal.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraResult.msg"
    "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/cmake" TYPE FILE FILES "/home/jack/SensorsAndControlA1/catkin_ws/build/fetch_gazebo/fetchit_challenge/catkin_generated/installspace/fetchit_challenge-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jack/SensorsAndControlA1/catkin_ws/devel/include/fetchit_challenge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/roseus/ros/fetchit_challenge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/common-lisp/ros/fetchit_challenge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/jack/SensorsAndControlA1/catkin_ws/devel/share/gennodejs/ros/fetchit_challenge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/jack/SensorsAndControlA1/catkin_ws/devel/lib/python2.7/dist-packages/fetchit_challenge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/jack/SensorsAndControlA1/catkin_ws/devel/lib/python2.7/dist-packages/fetchit_challenge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jack/SensorsAndControlA1/catkin_ws/build/fetch_gazebo/fetchit_challenge/catkin_generated/installspace/fetchit_challenge.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/cmake" TYPE FILE FILES "/home/jack/SensorsAndControlA1/catkin_ws/build/fetch_gazebo/fetchit_challenge/catkin_generated/installspace/fetchit_challenge-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/cmake" TYPE FILE FILES
    "/home/jack/SensorsAndControlA1/catkin_ws/build/fetch_gazebo/fetchit_challenge/catkin_generated/installspace/fetchit_challengeConfig.cmake"
    "/home/jack/SensorsAndControlA1/catkin_ws/build/fetch_gazebo/fetchit_challenge/catkin_generated/installspace/fetchit_challengeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge" TYPE FILE FILES "/home/jack/SensorsAndControlA1/catkin_ws/src/fetch_gazebo/fetchit_challenge/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/fetchit_challenge" TYPE PROGRAM FILES
    "/home/jack/SensorsAndControlA1/catkin_ws/src/fetch_gazebo/fetchit_challenge/scripts/spawn_assembly_delayed.sh"
    "/home/jack/SensorsAndControlA1/catkin_ws/src/fetch_gazebo/fetchit_challenge/scripts/spawn_assembly_delayed_simple.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge" TYPE DIRECTORY FILES
    "/home/jack/SensorsAndControlA1/catkin_ws/src/fetch_gazebo/fetchit_challenge/config"
    "/home/jack/SensorsAndControlA1/catkin_ws/src/fetch_gazebo/fetchit_challenge/launch"
    "/home/jack/SensorsAndControlA1/catkin_ws/src/fetch_gazebo/fetchit_challenge/models"
    "/home/jack/SensorsAndControlA1/catkin_ws/src/fetch_gazebo/fetchit_challenge/urdf"
    "/home/jack/SensorsAndControlA1/catkin_ws/src/fetch_gazebo/fetchit_challenge/worlds"
    )
endif()

