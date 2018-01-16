# Install script for directory: /home/emily/utilities/Flexiport/include/flexiport

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "library")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/flexiport-2/flexiport" TYPE FILE FILES
    "/home/emily/utilities/Flexiport/include/flexiport/flexiport.h"
    "/home/emily/utilities/Flexiport/include/flexiport/flexiport_types.h"
    "/home/emily/utilities/Flexiport/include/flexiport/logfile.h"
    "/home/emily/utilities/Flexiport/include/flexiport/logreaderport.h"
    "/home/emily/utilities/Flexiport/include/flexiport/logwriterport.h"
    "/home/emily/utilities/Flexiport/include/flexiport/port.h"
    "/home/emily/utilities/Flexiport/include/flexiport/serialport.h"
    "/home/emily/utilities/Flexiport/include/flexiport/tcpport.h"
    "/home/emily/utilities/Flexiport/include/flexiport/timeout.h"
    "/home/emily/utilities/Flexiport/include/flexiport/udpport.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "library")

