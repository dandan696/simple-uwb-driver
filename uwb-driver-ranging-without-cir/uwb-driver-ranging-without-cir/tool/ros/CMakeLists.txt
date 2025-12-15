cmake_minimum_required(VERSION 3.0)
project(simple_uwb_driver)

set(CMAKE_BUILD_TYPE "Release")

set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -Wno-unused-local-typedefs")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  serial
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_loam
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB SOURCES src/*/*.cc)

add_executable(simple_uwb_driver
    src/main.cc
)
target_link_libraries(simple_uwb_driver 
    ${catkin_LIBRARIES}
)

