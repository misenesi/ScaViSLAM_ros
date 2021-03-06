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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

IF(NOT CUDA_SUPPORT)
  SET(CUDA_SUPPORT ON)
ENDIF()
#
IF(CUDA_SUPPORT)
  MESSAGE ("CUDA SUPPORT ON")
  ADD_DEFINITIONS(-DSCAVISLAM_CUDA_SUPPORT)
ELSE()
  MESSAGE ("CUDA SUPPORT OFF")
ENDIF()

IF(CUDA_SUPPORT)
  LIST(APPEND INCLUDE_DIRS "EXTERNAL/opencv2/modules/gpu/include")
  FIND_PACKAGE(CUDA REQUIRED)
  SET(CUDA_NVCC_FLAGS  "-arch=sm_20" "--use_fast_math" "-O3"
                       "--ptxas-options=--verbose" "-keep"  )
  ADD_DEFINITIONS(-DCUDA_BUILD -DBT_USE_DOUBLE_PRECISION)
ENDIF()

# eIGEN ####################
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#cuda 
INCLUDE_DIRECTORIES(/usr/local/cuda/include)

rosbuild_add_executable(scavislam src/stereo_slam.cpp)
target_link_libraries(scavislam scavislam_lib g2o_csparse_extension ${OpenCV_LIBS})# /home/kir1pal/svslocal/lib/libboost_regex.so)# /home/kir1pal/svslocal/lib/libboost_thread.so)
#target_link_libraries(program kernel shrutil_x86_64 cutil_x86_64)

#CUDA_ADD_EXECUTABLE(scavislam src/stereo_slam.cpp src/ros_frame_grabber.cpp)
