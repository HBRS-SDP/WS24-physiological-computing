# /* ================================================================================
#  * Copyright: (C) 2024, Ayush Salunke, 
#  *       Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
#  * 
#  * Authors: Ayush Salunke ayush.salunke@smail.inf.h-brs.de
#  * 
#  * CopyPolicy: Released under the terms of the MIT License.
#  *      See the accompanying LICENSE file for details.
#  * ================================================================================
#  */

cmake_minimum_required(VERSION 3.8)
project(hriphysio_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add HriPhysioLib manually
set(HRIPHYSIO_LIB_PATH "/home/ayush/sdp_ws/src/WS24-physiological-computing/HriPhysioLib")

# Link HriPhysioLib dynamically
# add_library(HriPhysioLib SHARED IMPORTED)
# set_target_properties(HriPhysioLib PROPERTIES
#     IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/../HriPhysioLib/build/libHriPhysioLib.so
#     INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_SOURCE_DIR}/../HriPhysioLib
# )

include_directories(
  ${HRIPHYSIO_LIB_PATH}/include
  ${rclcpp_INCLUDE_DIRS}
)
link_directories(${HRIPHYSIO_LIB_PATH}/lib)

# Create ROS2 nodes using HriPhysioLib
add_executable(spectrogram_publisher_node src/spectrogram_publisher.cpp)
target_link_libraries(spectrogram_publisher_node ${HRIPHYSIO_LIB_PATH}/build/libHriPhysio.a)
ament_target_dependencies(spectrogram_publisher_node rclcpp std_msgs)

# Install targets
install(TARGETS
    spectrogram_publisher_node
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
