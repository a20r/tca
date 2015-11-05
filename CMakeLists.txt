cmake_minimum_required(VERSION 2.8.3)
project(tca)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-write-strings ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  nav_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs nav_msgs
)

add_subdirectory(resources/dynamicvoronoi dynamicvoronoi)

link_directories(/usr/local/lib)

include_directories(
    include ${catkin_INCLUDE_DIRS} resources/dynamicvoronoi/src
)

add_executable(tca_node src/tca.cpp src/pathgraph.cpp src/searchpath.cpp)
target_link_libraries(tca_node
    ${catkin_LIBRARIES} dynamicvoronoi)
