sudo killall gpsd
sudo rm /var/run/gpsd.sock
sudo systemctl stop gpsd.socket
ps -ef|grep gps|grep -v grep
sudo gpsd -b `python gps_port.py` -F /var/run/gpsd.sock
ps -ef|grep gps|grep -v grep
gpspipe -r