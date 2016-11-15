# Install script for directory: /auto/fsb/cramsted/Documents/EE483/Controls/lab6/whirlybird_ws/src/whirlybird_hardware

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/auto/fsb/cramsted/Documents/EE483/Controls/lab6/whirlybird_ws/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/auto/fsb/cramsted/Documents/EE483/Controls/lab6/whirlybird_ws/build/whirlybird_hardware/catkin_generated/installspace/whirlybird_hardware.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/whirlybird_hardware/cmake" TYPE FILE FILES
    "/auto/fsb/cramsted/Documents/EE483/Controls/lab6/whirlybird_ws/build/whirlybird_hardware/catkin_generated/installspace/whirlybird_hardwareConfig.cmake"
    "/auto/fsb/cramsted/Documents/EE483/Controls/lab6/whirlybird_ws/build/whirlybird_hardware/catkin_generated/installspace/whirlybird_hardwareConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/whirlybird_hardware" TYPE FILE FILES "/auto/fsb/cramsted/Documents/EE483/Controls/lab6/whirlybird_ws/src/whirlybird_hardware/package.xml")
endif()

