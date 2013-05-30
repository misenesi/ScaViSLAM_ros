cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

### CUDA ###############################
IF(NOT CUDA_SUPPORT)
  SET(CUDA_SUPPORT ON)
ENDIF()

IF(CUDA_SUPPORT)
  MESSAGE ("CUDA SUPPORT ON")
  ADD_DEFINITIONS(-DSCAVISLAM_CUDA_SUPPORT)
ELSE()
  MESSAGE ("CUDA SUPPORT OFF")
ENDIF()

IF(CUDA_SUPPORT)
  #LIST(APPEND INCLUDE_DIRS "EXTERNAL/opencv2/modules/gpu/include")
  FIND_PACKAGE(CUDA REQUIRED)
  SET(CUDA_NVCC_FLAGS  "-arch=sm_20" "--use_fast_math" "-O3"
                       "--ptxas-options=--verbose" "-keep"  )
  ADD_DEFINITIONS(-DCUDA_BUILD -DBT_USE_DOUBLE_PRECISION)
ENDIF()

### Eigen  #############################
FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})

# Iterate through other required libraries
SET (LIB_NAMES GL glut boost_system boost_thread boost_filesystem boost_regex)

FOREACH(lib ${LIB_NAMES})
  FIND_LIBRARY(LIB_${lib} ${lib})
  IF (LIB_${lib})
    MESSAGE(STATUS "found library '${lib}': ${LIB_${lib}}")
  ELSE (LIB_${lib})
    MESSAGE(FATAL_ERROR "cannot find library '${lib}' - this will not work ...")
  ENDIF (LIB_${lib})
  LIST(APPEND LIBS ${LIB_${lib}})
ENDFOREACH(lib)

#Add class XYZ here which consits of two files: XYZ.cpp and XYZ.h
SET (CLASSES  placerecognizer
              maths_utils
              ransac_models
              homography
              g2o_types/anchored_points
              g2o_types/register_types
              stereo_camera
              backend
              fast_grid
              stereo_frontend
              dense_tracking
              filegrabber)


SET (TEMPLATE ransac
              frame_grabber
              matcher
              slam_graph )

SET (SOURCE_DIR "src")
SET (SOURCES ${SOURCE_DIR}/draw_items.h
             ${SOURCE_DIR}/global.h   
             ${SOURCE_DIR}/keyframes.h
             ${SOURCE_DIR}/quadtree.h
             ${SOURCE_DIR}/transformations.h
             ${SOURCE_DIR}/data_structures.h
             ${SOURCE_DIR}/pose_optimizer.h)

IF(CUDA_SUPPORT)
  LIST(APPEND SOURCES ${SOURCE_DIR}/gpu/dense_tracking.cuh
                      ${SOURCE_DIR}/gpu/dense_tracking.cu)
ENDIF()

FOREACH(class ${CLASSES})
  LIST(APPEND SOURCES ${SOURCE_DIR}/${class}.cpp ${SOURCE_DIR}/${class}.h)
ENDFOREACH(class)

FOREACH(template ${TEMPLATE})
  LIST(APPEND SOURCES ${SOURCE_DIR}/${template}.cpp   
                      ${SOURCE_DIR}/${template}.hpp
                      ${SOURCE_DIR}/${template}-impl.cpp )
ENDFOREACH(template)

LINK_DIRECTORIES(${LINK_DIRS})
LINK_LIBRARIES (${PROJECT_NAME} ${LIBS} ${CSPARSE_LIBRARY})

IF(CUDA_SUPPORT)
  LIST(APPEND INCLUDE_DIRS ${CUDA_SDK_ROOT_DIR}/common/inc ${CUDA_INCLUDE_DIRS})
#  CUDA_ADD_EXECUTABLE(stereo_slam ${SOURCE_DIR}/stereo_slam.cpp)
  CUDA_ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
ELSE()
  #ADD_EXECUTABLE(stereo_slam ${SOURCE_DIR}/stereo_slam.cpp)
  ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
ENDIF()
