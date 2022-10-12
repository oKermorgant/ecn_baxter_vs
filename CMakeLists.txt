cmake_minimum_required(VERSION 2.8.3)
project(ecn_baxter_vs)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

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
find_package(log2plot REQUIRED)

set(CMAKE_CXX_STANDARD 17)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ecn_baxter_vs
  CATKIN_DEPENDS baxter_core_msgs roscpp sensor_msgs std_msgs image_transport ecn_common
  DEPENDS VISP OpenCV log2plot
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
  ${LOG2PLOT_INCLUDE_DIRS}
)

add_library(ecn_baxter_vs include/ecn_baxter_vs/baxter_arm.h src/baxter_arm.cpp)
target_link_libraries(ecn_baxter_vs ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${VISP_LIBRARIES} ${LOG2PLOT_LIBRARIES})

add_executable(baxter_vs main.cpp)
target_link_libraries(baxter_vs ecn_baxter_vs)
