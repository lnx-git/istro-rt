@SET PATH=%PATH%;C:\Programs\MinGW\bin
mingw32-g++ -c config.cpp
mingw32-g++ -c system.cpp 
mingw32-g++ -c threads.cpp
mingw32-g++ -c navig.cpp
mingw32-g++ -c -I"C:\Programs\opencv\include" gpsdev.cpp
mingw32-g++ -c -I"C:\Programs\opencv\include" geocalc.cpp
mingw32-g++ -c -I"C:\Programs\opencv\include" ctrlboard.cpp 
mingw32-g++ -c -I"C:\Programs\opencv\include" lidar.cpp 
mingw32-g++ -c -I"C:\Programs\opencv\include" camera.cpp 
mingw32-g++ -c -I"C:\Programs\opencv\include" vision.cpp 
mingw32-g++ -c -I"C:\Programs\opencv\include" sample.cpp
mingw32-g++ -c -I"C:\Programs\opencv\include" myahrs.cpp
mingw32-g++ -c -I"C:\Programs\opencv\include" dataset.cpp
mingw32-g++ -I"C:\Programs\opencv\include" istro_rt2016.cpp -L"C:\Programs\opencv\x86\mingw\staticlib" ctrlboard.o lidar.o camera.o system.o vision.o sample.o dataset.o threads.o gpsdev.o geocalc.o myahrs.o config.o navig.o -lopencv_core2413 -lopencv_highgui2413 -lopencv_imgproc2413 -llibjpeg -llibpng -llibtiff -lzlib -lgdi32 -lpthread -o istro_rt2016
@rem  > __log 2> __err
