cmake_minimum_required(VERSION 2.8.3)
project(reflector_ekf_slam)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  geometry_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  pcl_ros
)
catkin_package()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
src/reflector_ekf_slam.cc
)

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${EIGEN3_LIBS}
)

add_executable(slam_node
test/slam_node.cc)
target_link_libraries(slam_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)