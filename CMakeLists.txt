cmake_minimum_required(VERSION 3.5)
project(ecn_baxter_vs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
find_package(log2plot REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${LOG2PLOT_INCLUDE_DIRS}
)

ament_auto_add_library(ecn_baxter_vs src/baxter_arm.cpp src/color_detector.cpp)
target_link_libraries(ecn_baxter_vs ${OpenCV_LIBRARIES} ${VISP_LIBRARIES} ${LOG2PLOT_LIBRARIES})

ament_auto_add_executable(baxter_vs main.cpp)
target_link_libraries(baxter_vs ecn_baxter_vs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES src/ball_motion.py DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(urdf tf2_ros image_transport)
ament_export_libraries(ecn_baxter_vs)
ament_export_include_directories(include)

ament_auto_package()
