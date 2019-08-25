cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

project(KeyToResetModel)

find_package(gazebo REQUIRED)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_NAME}.cc
)
target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
