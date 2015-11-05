cmake_minimum_required(VERSION 2.6)
project(DYNAMICVORONOI)

# SET(CMAKE_BUILD_TYPE Release)

# COMPILER FLAGS
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb")

include_directories(src)

add_library(dynamicvoronoi src/dynamicvoronoi.cpp)

add_executable(example src/example.cpp)
target_link_libraries(example dynamicvoronoi)

