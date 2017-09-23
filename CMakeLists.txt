project( istro_rt2017 )
cmake_minimum_required(VERSION 2.8)

# CONFIGURE OPENCV
find_package( OpenCV REQUIRED )
find_package( GeographicLib 1.34 REQUIRED )

# CREATE EXECUTABLE
SET(CMAKE_CXX_COMPILER "g++-5")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -Wall")
add_definitions( ${GeographicLib_DEFINITIONS} )
add_executable( istro_rt2017 istro_rt2017.cpp system.cpp ctrlboard.cpp camera.cpp vision.cpp mtime.cpp sample.cpp dmap.cpp lidar.cpp threads.cpp gpsdev.cpp geocalc.cpp myahrs.cpp config.cpp navig.cpp wmodel.cpp dataset.cpp )
target_link_libraries( istro_rt2017 ${OpenCV_LIBS} ${GeographicLib_LIBRARIES} /home/pi/projects/rplidar_sdk_v1.4.5/sdk/output/Linux/Release/librplidar_sdk.a stdc++ pthread gps log4cxx )
include_directories( ${GeographicLib_INCLUDE_DIRS} /home/pi/projects/rplidar_sdk_v1.4.5/sdk/sdk/include )
                                                                                                                                                                                 
