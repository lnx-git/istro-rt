sudo apt-get install gpsd gpsd-clients python-gps
sudo systemctl stop gpsd.socket
sudo systemctl disable gpsd.socket
sudo apt-get install libgps-dev
