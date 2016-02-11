cmake_minimum_required(VERSION 2.8.3)
project(tca)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-write-strings ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-fopenmp ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -flto")

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

add_subdirectory(third_party/dynamicvoronoi dynamicvoronoi)

include_directories(
    include ${catkin_INCLUDE_DIRS} third_party/dynamicvoronoi/src
)

add_executable(tca_node src/tca_node.cpp src/voronoi.cpp src/graph.cpp src/json.cpp
    src/structures.cpp src/io.cpp)
target_link_libraries(tca_node ${catkin_LIBRARIES} dynamicvoronoi)
