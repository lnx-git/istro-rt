project( istro_rt2020 )
cmake_minimum_required(VERSION 2.8)

# CONFIGURE OPENCV
find_package( OpenCV REQUIRED )
find_package( GeographicLib 1.34 REQUIRED )
find_package( LibXml2 REQUIRED )

# CREATE EXECUTABLE
SET(CMAKE_CXX_COMPILER "g++")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -Wall")
add_definitions( ${GeographicLib_DEFINITIONS} ${LIBXML2_DEFINITIONS} )
add_executable( istro_rt2020 istro_rt2020.cpp system.cpp ctrlboard.cpp camera.cpp vision.cpp visionn.cpp vision_depth.cpp mtime.cpp sample.cpp dmap.cpp lidar.cpp threads.cpp gpsdev.cpp geocalc.cpp myahrs.cpp config.cpp navig.cpp navig_data.cpp wmodel.cpp dataset.cpp qrscan.cpp navmap.cpp navmap_data.cpp )
target_link_libraries( istro_rt2020 ${OpenCV_LIBS} ${GeographicLib_LIBRARIES} ${LIBXML2_LIBRARIES} /usr/local/rplidar/sdk/output/Linux/Release/librplidar_sdk.a stdc++ pthread gps log4cxx zbar realsense2 )
include_directories( ${GeographicLib_INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR} /usr/local/rplidar/sdk/sdk/include )
