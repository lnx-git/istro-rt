@SET PATH=%PATH%;C:\Programs\MinGW\bin
mingw32-g++ -Wall -c dmap.cpp
mingw32-g++ -Wall -c config.cpp
mingw32-g++ -Wall -c threads.cpp
mingw32-g++ -Wall -c logger.cpp
mingw32-g++ -Wall -c navig.cpp
mingw32-g++ -Wall -c navig_data.cpp
mingw32-g++ -Wall -c system.cpp 
mingw32-g++ -Wall -c gpsdev.cpp
mingw32-g++ -Wall -c geocalc.cpp
mingw32-g++ -Wall -c myahrs.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" wmodel.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" mtime.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" ctrlboard.cpp 
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" lidar.cpp 
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" camera.cpp 
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" sample.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" vision.cpp 
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" visionn.cpp 
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" vision_depth.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" dataset.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" qrscan.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" navmap_data.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv3\include" -Wformat=0 navmap.cpp
mingw32-g++ -Wall -I"C:\Programs\opencv3\include" istro_rt2020.cpp -L"C:\Programs\opencv3\x86\mingw\staticlib" ctrlboard.o lidar.o camera.o system.o vision.o visionn.o vision_depth.o mtime.o sample.o dataset.o threads.o logger.o gpsdev.o geocalc.o myahrs.o config.o navig.o navig_data.o dmap.o wmodel.o qrscan.o navmap.o navmap_data.o -lopencv_highgui342 -lopencv_imgcodecs342 -lopencv_imgproc342 -lopencv_core342 -lzlib -llibjpeg-turbo -llibwebp -llibpng -llibtiff -lgdi32 -lpthread -lcomdlg32 -o istro_rt2020
@rem  > __log 2> __err
