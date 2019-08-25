cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(SetStaticPose)

find_package(gazebo REQUIRED)
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(SetStaticPose SHARED SetStaticPose.cc)
target_link_libraries(SetStaticPose ${GAZEBO_LIBRARIES})
