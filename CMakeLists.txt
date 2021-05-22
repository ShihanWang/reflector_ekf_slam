cmake_minimum_required(VERSION 2.8.3)
project(reflector_ekf_slam)

add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(PACKAGE_DEPENDENCIES
  visualization_msgs
  nav_msgs
  geometry_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(Glog REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

# find_package(CAIRO REQUIRED cairo>=1.12.16)
include(FindPkgConfig)
PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (NOT GLOG_FOUND)
    message(FATAL_ERROR "Glog not found. Install Glog from the command line using the command(s) -\
      sudo apt-get install libgoogle-glog-dev libgflags-dev")
endif (NOT GLOG_FOUND)

add_service_files(
   FILES
   save_map.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    PCL
    EIGEN3
)

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${GLOG_INCLUDE_DIR}
${CERES_INCLUDE_DIRS}
${CAIRO_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "include/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "src/*.cc" "src/*.cpp")

add_executable(slam_node
${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})

target_link_libraries(slam_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  glog
  ${CERES_LIBRARIES}
  ${CAIRO_LIBRARIES}
)