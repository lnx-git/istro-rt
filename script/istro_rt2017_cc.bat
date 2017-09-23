@SET PATH=%PATH%;C:\Programs\MinGW\bin
mingw32-g++ -Wall -c dmap.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv\include" wmodel.cpp
mingw32-g++ -Wall -c config.cpp
mingw32-g++ -Wall -c threads.cpp
mingw32-g++ -Wall -c logger.cpp
mingw32-g++ -Wall -c navig.cpp
mingw32-g++ -Wall -c system.cpp 
mingw32-g++ -Wall -c gpsdev.cpp
mingw32-g++ -Wall -c geocalc.cpp
mingw32-g++ -Wall -c myahrs.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv\include" mtime.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv\include" ctrlboard.cpp 
mingw32-g++ -Wall -c -I"C:\Programs\opencv\include" lidar.cpp 
mingw32-g++ -Wall -c -I"C:\Programs\opencv\include" camera.cpp 
mingw32-g++ -Wall -c -I"C:\Programs\opencv\include" sample.cpp
mingw32-g++ -Wall -c -I"C:\Programs\opencv\include" vision.cpp 
mingw32-g++ -Wall -c -I"C:\Programs\opencv\include" dataset.cpp
mingw32-g++ -Wall -I"C:\Programs\opencv\include" istro_rt2017.cpp -L"C:\Programs\opencv\x86\mingw\staticlib" ctrlboard.o lidar.o camera.o system.o vision.o mtime.o sample.o dataset.o threads.o logger.o gpsdev.o geocalc.o myahrs.o config.o navig.o dmap.o wmodel.o -lopencv_core2413 -lopencv_highgui2413 -lopencv_imgproc2413 -llibjpeg -llibpng -llibtiff -lzlib -lgdi32 -lpthread -o istro_rt2017
@rem  > __log 2> __err
