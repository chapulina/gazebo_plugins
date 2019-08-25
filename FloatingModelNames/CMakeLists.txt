cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(FloatingModelNames)

find_package (Qt4)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

QT4_WRAP_CPP(headers_MOC FloatingModelNames.hh)

add_library(floating_model_names SHARED ${headers_MOC}
  FloatingModelNames.cc)
target_link_libraries(floating_model_names ${GAZEBO_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
