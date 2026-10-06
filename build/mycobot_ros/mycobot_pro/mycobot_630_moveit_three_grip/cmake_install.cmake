# Install script for directory: /home/stavros/catkin_ws/src/mycobot_ros/mycobot_pro/mycobot_630_moveit_three_grip

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/stavros/catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/stavros/catkin_ws/build/mycobot_ros/mycobot_pro/mycobot_630_moveit_three_grip/catkin_generated/installspace/mycobot_630_moveit_three_grip.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mycobot_630_moveit_three_grip/cmake" TYPE FILE FILES
    "/home/stavros/catkin_ws/build/mycobot_ros/mycobot_pro/mycobot_630_moveit_three_grip/catkin_generated/installspace/mycobot_630_moveit_three_gripConfig.cmake"
    "/home/stavros/catkin_ws/build/mycobot_ros/mycobot_pro/mycobot_630_moveit_three_grip/catkin_generated/installspace/mycobot_630_moveit_three_gripConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mycobot_630_moveit_three_grip" TYPE FILE FILES "/home/stavros/catkin_ws/src/mycobot_ros/mycobot_pro/mycobot_630_moveit_three_grip/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mycobot_630_moveit_three_grip" TYPE DIRECTORY FILES "/home/stavros/catkin_ws/src/mycobot_ros/mycobot_pro/mycobot_630_moveit_three_grip/launch" REGEX "/setup\\_assistant\\.launch$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mycobot_630_moveit_three_grip" TYPE DIRECTORY FILES "/home/stavros/catkin_ws/src/mycobot_ros/mycobot_pro/mycobot_630_moveit_three_grip/config")
endif()

