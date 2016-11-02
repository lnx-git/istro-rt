sudo killall gpsd
sudo rm /var/run/gpsd.sock
sudo systemctl stop gpsd.socket
ps -ef|grep gps|grep -v grep
