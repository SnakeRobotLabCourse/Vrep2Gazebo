cmake_minimum_required(VERSION 2.8.3)
project(robosnake_simplegait)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  std_msgs
  roscpp
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${gazebo_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS} 
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(simplegait SHARED src/simplegait.cpp src/gait.cpp)
target_link_libraries(simplegait ${GAZEBO_libraries} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} SHARED src/robosnake_simplegait.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_libraries}
  ${roscpp_LIBRARIES}
  simplegait
)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
  INCLUDE_DIRS include
)
