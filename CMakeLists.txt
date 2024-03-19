cmake_minimum_required(VERSION 3.0.2)
project(livox_360_processing)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS "-std=gnu++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)


add_executable(interpolated_livox360_node  src/interpolated_livox.cpp)
target_link_libraries(interpolated_livox360_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)








