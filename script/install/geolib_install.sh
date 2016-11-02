wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.46.tar.gz
tar xfpz GeographicLib-1.46.tar.gz
cd GeographicLib-1.46
mkdir BUILD
cd BUILD
cmake ..
make
make test
sudo make install
