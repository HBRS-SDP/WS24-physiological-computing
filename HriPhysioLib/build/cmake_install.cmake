# Install script for directory: /home/prachisheth/WS24-physiological-computing/HriPhysioLib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/build/libHriPhysio.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/HriPhysio/ringBuffer.h;/HriPhysio/graph.h;/HriPhysio/streamerFactory.h;/HriPhysio/physioManager.h;/HriPhysio/robotManager.h;/HriPhysio/threadManager.h;/HriPhysio/biquadratic.h;/HriPhysio/butterworthBandNoch.h;/HriPhysio/butterworthBandPass.h;/HriPhysio/butterworthHighPass.h;/HriPhysio/butterworthLowPass.h;/HriPhysio/hilbertTransform.h;/HriPhysio/math.h;/HriPhysio/spectrogram.h;/HriPhysio/robotInterface.h;/HriPhysio/csvStreamer.h;/HriPhysio/lslStreamer.h;/HriPhysio/streamerInterface.h;/HriPhysio/helpers.h;/HriPhysio/pocketfft.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/HriPhysio" TYPE FILE FILES
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Core/ringBuffer.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Core/graph.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Factory/streamerFactory.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Manager/physioManager.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Manager/robotManager.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Manager/threadManager.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Processing/biquadratic.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Processing/butterworthBandNoch.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Processing/butterworthBandPass.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Processing/butterworthHighPass.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Processing/butterworthLowPass.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Processing/hilbertTransform.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Processing/math.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Processing/spectrogram.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Social/robotInterface.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Stream/csvStreamer.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Stream/lslStreamer.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/Stream/streamerInterface.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/HriPhysio/helpers.h"
    "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/include/PocketFFT/pocketfft.h"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/build/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/prachisheth/WS24-physiological-computing/HriPhysioLib/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
