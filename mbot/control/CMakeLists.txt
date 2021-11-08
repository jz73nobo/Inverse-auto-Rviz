cmake_minimum_required(VERSION 3.0.2)
project(control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   gazebo_ros
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
     roscpp
     rospy
     gazebo_ros
)

## Include Directories
include_directories(
   ${catkin_INCLUDE_DIRS}
   ${src}
)

add_executable(control src/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES})
add_dependencies(control ${catkin_EXPORTED_TARGETS})

add_executable(dynamic src/dynamic.cpp)
target_link_libraries(dynamic ${catkin_LIBRARIES})
add_dependencies(dynamic ${catkin_EXPORTED_TARGETS})
