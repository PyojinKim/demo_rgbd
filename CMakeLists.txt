cmake_minimum_required(VERSION 2.8.3)
project(demo_rgbd)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(SENSE_LIB_DIR "/home/icsl/Documents/librealsense/build" CACHE PATH " ")
SET(Boost_USE_MULTITHREADED ON)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  pcl_ros
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)


## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)


add_subdirectory(isamlib)
link_libraries(isamlib)


# Set link libraries
LIST(APPEND LINK_LIBS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${SENSE_LIB_DIR}/librealsense.so
)


# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
set(CMAKE_CXX_FLAGS "-msse3")
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -Ofast -flto -march=native")


################################################################################
# TESTS

## Declare executables
add_executable(realsenseTopics src/realsenseTopics.cpp)
add_executable(realsenseTopicsSubscriber src/realsenseTopicsSubscriber.cpp)
add_executable(TUMdatasetTopicConverter src/TUMdatasetTopicConverter.cpp)
add_executable(extractImageBag src/extractImageBag.cpp)
add_executable(featureTracking src/featureTracking.cpp)
add_executable(visualOdometry src/visualOdometry.cpp)
add_executable(bundleAdjust src/bundleAdjust.cpp)
add_executable(processDepthmap src/processDepthmap.cpp)
add_executable(stackDepthPoint src/stackDepthPoint.cpp)
add_executable(transformMaintenance src/transformMaintenance.cpp)
add_executable(registerPointCloud src/registerPointCloud.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(realsenseTopics ${LINK_LIBS})
target_link_libraries(realsenseTopicsSubscriber ${LINK_LIBS})
target_link_libraries(TUMdatasetTopicConverter ${LINK_LIBS})
target_link_libraries(extractImageBag ${LINK_LIBS})
target_link_libraries(featureTracking ${LINK_LIBS})
target_link_libraries(visualOdometry ${LINK_LIBS})
target_link_libraries(bundleAdjust ${LINK_LIBS})
target_link_libraries(processDepthmap ${LINK_LIBS})
target_link_libraries(stackDepthPoint ${LINK_LIBS})
target_link_libraries(transformMaintenance ${LINK_LIBS})
target_link_libraries(registerPointCloud ${LINK_LIBS})
