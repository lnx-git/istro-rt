sudo killall gpsd
sudo rm /var/run/gpsd.sock
sudo systemctl stop gpsd.socket
ps -ef|grep gps|grep -v grep
sudo gpsd -b /dev/ttyUSB3 -F /var/run/gpsd.sock
ps -ef|grep gps|grep -v grep
gpspipe -r -n 20
