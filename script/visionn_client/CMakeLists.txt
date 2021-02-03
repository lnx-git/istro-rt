project( visionn )
cmake_minimum_required(VERSION 2.8)

# CONFIGURE OPENCV
find_package( OpenCV REQUIRED )

# CREATE EXECUTABLE
SET(CMAKE_CXX_COMPILER "g++")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -Wall")
add_executable( visionn visionn.cpp )
target_link_libraries( visionn ${OpenCV_LIBS} stdc++ )
include_directories( )

