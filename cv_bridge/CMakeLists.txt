cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# opencv ####################
find_package(OpenCV 2.4.2 REQUIRED HINTS ~/svslocal/share)
include_directories(${OpenCV_INCLUDE_DIRS})
message("##########OPENCV INCS#############" ${OpenCV_INCLUDE_DIRS})
message("##########OPENCV LIBS#############" ${OpenCV_LIBS})

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/cv_bridge.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
