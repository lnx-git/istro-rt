cd opencv-2.4.13
sudo make uninstall
sudo rm -r /usr/local/share/OpenCV/
sudo rm -r /usr/local/include/opencv
sudo rm -r /usr/local/include/opencv2
sudo rm /usr/local/lib/libopencv_*
cd ..

mkdir opencv-3.4.2
cd opencv-3.4.2
wget -O opencv-3.4.2.zip https://codeload.github.com/opencv/opencv/zip/3.4.2
unzip opencv-3.4.2.zip 
mv opencv-3.4.2 opencv3
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_C_EXAMPLES=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -D BUILD_EXAMPLES=OFF opencv3
make -j4
sudo make install
