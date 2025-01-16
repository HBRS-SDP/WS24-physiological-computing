# Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, University of Waterloo
# Authors: Austin Kothig <austin.kothig@uwaterloo.ca>
# CopyPolicy: Released under the terms of the BSD 3-Clause License.

# /* ================================================================================
#  * Copyright: (C) 2024, Ayush Salunke,
#  *     Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
#  * 
#  * Author: 
#  *     Ayush Salunke<ayush.salunke@smail.inf.h-brs.de>
#  * 
#  * CopyPolicy: Released under the terms of the MIT License.
#  *     See the accompanying LICENSE file for details.
#  * ================================================================================
#  */

#  /* ================================================================================
#  * Copyright: (C) 2024, Prachi Sheth,
#  *     Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
#  * 
#  * Author: 
#  *     Prachi Sheth <prachi.sheth@smail.inf.h-brs.de>
#  * 
#  * CopyPolicy: Released under the terms of the MIT License.
#  *     See the accompanying LICENSE file for details.
#  * ================================================================================
#  */

#  /* ================================================================================
#  * Copyright: (C) 2024, Yash Somaiya,
#  *     Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
#  * 
#  * Author: 
#  *     Yash Somaiya <yash.somaiya@smail.inf.h-brs.de>
#  * 
#  * CopyPolicy: Released under the terms of the MIT License.
#  *     See the accompanying LICENSE file for details.
#  * ================================================================================
#  */

cmake_minimum_required(VERSION 3.12)

# Define the project name and version
project(HriPhysio VERSION 1.0)

# Generate compile_commands.json for clang-tidy and other tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED YES)

# Set the library target name
set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

############################################################

# List of CPP (source) library files
set(${LIBRARY_TARGET_NAME}_SRC
    # Core
    src/Core/graph.cpp

    # Factory
    src/Factory/streamerFactory.cpp

    # Manager
    src/Manager/robotManager.cpp
    src/Manager/threadManager.cpp
    src/Manager/physioManager.cpp

    # Processing
    src/Processing/biquadratic.cpp
    src/Processing/butterworthBandNoch.cpp
    src/Processing/butterworthBandPass.cpp
    src/Processing/butterworthHighPass.cpp
    src/Processing/butterworthLowPass.cpp
    src/Processing/hilbertTransform.cpp
    src/Processing/spectrogram.cpp

    # Social
    src/Social/robotInterface.cpp

    # Stream
    src/Stream/streamerInterface.cpp
    src/Stream/lslStreamer.cpp
    src/Stream/csvStreamer.cpp
    src/Stream/main.cpp

    # HELPERS
    src/helpers.cpp
)

# List of HPP (header) library files
set(${LIBRARY_TARGET_NAME}_HDR
    # CORE
    include/HriPhysio/Core/ringBuffer.h
    include/HriPhysio/Core/graph.h

    # FACTORY
    include/HriPhysio/Factory/streamerFactory.h

    # MANAGER
    include/HriPhysio/Manager/physioManager.h
    include/HriPhysio/Manager/robotManager.h
    include/HriPhysio/Manager/threadManager.h

    # PROCESSING
    include/HriPhysio/Processing/biquadratic.h
    include/HriPhysio/Processing/butterworthBandNoch.h
    include/HriPhysio/Processing/butterworthBandPass.h
    include/HriPhysio/Processing/butterworthHighPass.h
    include/HriPhysio/Processing/butterworthLowPass.h
    include/HriPhysio/Processing/hilbertTransform.h
    include/HriPhysio/Processing/math.h
    include/HriPhysio/Processing/spectrogram.h

    # SOCIAL
    include/HriPhysio/Social/robotInterface.h

    # STREAM
    include/HriPhysio/Stream/csvStreamer.h
    include/HriPhysio/Stream/lslStreamer.h
    include/HriPhysio/Stream/streamerInterface.h

    # HELPERS
    include/HriPhysio/helpers.h

    # Additional Library Headers
    include/PocketFFT/pocketfft.h
)

############################################################

# Check if YARP is enabled
if(ENABLE_YARP)
    find_package(YARP REQUIRED)

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC
        src/Stream/yarpStreamer.cpp
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_HDR
        include/HriPhysio/Stream/yarp/yarpStreamer.h
    )
endif(ENABLE_YARP)

# Check if ROS is enabled
if(ENABLE_ROS)
    find_package(roscpp REQUIRED)
    find_package(std_msgs REQUIRED)

    include_directories(/opt/ros/$ENV{ROS_DISTRO}/include /opt/ros/$ENV{ROS_DISTRO}/lib)

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC
        src/Stream/rosStreamer.cpp
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_HDR
        include/HriPhysio/Stream/ros/rosStreamer.h
    )
endif(ENABLE_ROS)

############################################################

# Add the library target
add_library(
    ${LIBRARY_TARGET_NAME}
    ${${LIBRARY_TARGET_NAME}_SRC}
    ${${LIBRARY_TARGET_NAME}_HDR}
)

# Set library properties such as version and public header files
set_target_properties(
    ${LIBRARY_TARGET_NAME}
    PROPERTIES VERSION ${${PROJECT_NAME}_VERSION}
               PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}"
)

# Specify include directories for the library
target_include_directories(
    ${LIBRARY_TARGET_NAME}
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
           "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
)

# Link to other necessary libraries
target_link_libraries(
    ${LIBRARY_TARGET_NAME}
    pthread
    lsl
    yaml-cpp
)

# If ROS is enabled, link against ROS libraries
if(ENABLE_ROS)
    target_link_libraries(
        ${LIBRARY_TARGET_NAME}
        -I/opt/ros/$ENV{ROS_DISTRO}/include -L/opt/ros/$ENV{ROS_DISTRO}/lib
        -lroscpp -lrostime -lrosconsole -lroscpp_serialization
    )
endif(ENABLE_ROS)

###################################
# Added csvStreamer exe for testing
add_executable(csvStreamer src/Stream/main.cpp  src/Stream/csvStreamer.cpp)
target_link_libraries(csvStreamer ${LIBRARY_TARGET_NAME})

###################################

# Specify installation targets
install(
    TARGETS ${LIBRARY_TARGET_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}"
)

# Print a message when the target is created
message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")

