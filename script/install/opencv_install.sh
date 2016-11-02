sudo apt-get install build-essential cmake pkg-config
sudo apt-get install libgtk2.0-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libjpeg-dev libtiff-dev libjasper-dev libpng-dev
sudo apt-get install python2.7-dev python-numpy
cd ~pi
wget -O opencv-2.4.13.zip https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.13/opencv-2.4.13.zip/download
unzip opencv-2.4.13.zip
cd opencv-2.4.13
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_NEW_PYTHON_SUPPORT=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON  -D BUILD_EXAMPLES=ON ..
make
sudo make install
