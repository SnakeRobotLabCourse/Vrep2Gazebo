cmake_minimum_required(VERSION 2.8.3)
project(robosnake_simplegait)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS} ${gazebo_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} SHARED src/robosnake_simplegait.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  #${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_libraries}
)

#add_library(robosnake_simplegait SHARED robosnake_simplegait.cpp)
#target_link_libraries(robosnake_simplegait ${GAZEBO_libraries})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)
