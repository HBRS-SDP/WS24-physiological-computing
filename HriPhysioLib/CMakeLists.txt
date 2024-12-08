# Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, University of Waterloo
# Authors: Austin Kothig <austin.kothig@uwaterloo.ca>
# CopyPolicy: Released under the terms of the BSD 3-Clause License.

cmake_minimum_required( VERSION 3.12 )

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

############################################################

# List of CPP (source) library files.

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

    # HELPERS
    src/helpers.cpp
)


# List of HPP (header) library files.

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
    # include/HriPhysio/Stream/udpStreamer.h       
    
    # HELPERS
    include/HriPhysio/helpers.h

    # Additional Library Headers
    include/PocketFFT/pocketfft.h
)

############################################################

if( ENABLE_YARP )

    find_package( YARP REQUIRED )
    

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC
        src/Stream/yarpStreamer.cpp
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_HDR
        include/HriPhysio/Stream/yarp/yarpStreamer.h
    )

endif( ENABLE_YARP )

if( ENABLE_ROS )

    find_package( roscpp   REQUIRED )
    find_package( std_msgs REQUIRED )

    include_directories(/opt/ros/$ENV{ROS_DISTRO}/include /opt/ros/$ENV{ROS_DISTRO}/lib)


    list(APPEND ${LIBRARY_TARGET_NAME}_SRC
        src/Stream/rosStreamer.cpp
    )
    
    list(APPEND ${LIBRARY_TARGET_NAME}_HDR
        include/HriPhysio/Stream/ros/rosStreamer.h
    )

endif( ENABLE_ROS )

############################################################

# Adds a library target called ${LIBRARY_TARGET_NAME} to be built from the
# source and header files listed in the command invocation.

add_library(
    ${LIBRARY_TARGET_NAME} 
    ${${LIBRARY_TARGET_NAME}_SRC} 
    ${${LIBRARY_TARGET_NAME}_HDR}
)


# Within this project, you can link to this library by just specifing the name
# of the target. It is useful, however, to define an alias of this library with 
# the scope of the exported library itself because (1) you will link against it 
# with the exact same syntax of an imported library and (2) because names having 
# a double-colon (::) are always treated as the name of either an alias or 
# imported target. Any attempt to use such a name for a different target type 
# will result in an error.

add_library(
    ${PROJECT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME}
)


# Set two minimum target properties for the library.
# See https://cmake.org/cmake/help/latest/command/set_target_properties.html
# Properties are: 1) Library version number 2) list of corresponding public headers

set_target_properties(
    ${LIBRARY_TARGET_NAME} 
    PROPERTIES VERSION    ${${PROJECT_NAME}_VERSION}
    PUBLIC_HEADER        "${${LIBRARY_TARGET_NAME}_HDR}"
)


# Specify include directories for both compilation and installation process.
# The $<INSTALL_PREFIX> generator expression is useful to ensure to create
# relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages

target_include_directories(
    ${LIBRARY_TARGET_NAME} 
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
           "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
)


# If find_package() is used need target_include_directories() 
# and/or target_link_libraries() here.
target_link_libraries(
    ${LIBRARY_TARGET_NAME} 
    pthread
    lsl
    yaml-cpp
)


if( ENABLE_ROS )

    target_link_libraries(
        ${LIBRARY_TARGET_NAME} 
        -I/opt/ros/$ENV{ROS_DISTRO}/include -L/opt/ros/$ENV{ROS_DISTRO}/lib
        -lroscpp -lrostime -lrosconsole -lroscpp_serialization
    )

endif( ENABLE_ROS )


# Specify installation targets, typology and destination folders.

install(
    TARGETS       ${LIBRARY_TARGET_NAME}
    EXPORT        ${PROJECT_NAME}
    LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
    ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
    RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}" COMPONENT dev
)

############################################################

message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")

############################################################
