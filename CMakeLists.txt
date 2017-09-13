cmake_minimum_required(VERSION 2.8.3)
project(ecn_baxter_vs)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  urdf
  ecn_common
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ecn_baxter_vs
  CATKIN_DEPENDS baxter_core_msgs roscpp sensor_msgs std_msgs image_transport ecn_common
  DEPENDS VISP OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)

add_library(ecn_baxter_vs include/ecn_baxter_vs/baxter_arm.h src/baxter_arm.cpp)
target_link_libraries(ecn_baxter_vs ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${VISP_LIBRARIES})
