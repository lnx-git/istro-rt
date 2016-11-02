sudo gpsd -b /dev/ttyUSB2 -F /var/run/gpsd.sock
ps -ef|grep gps|grep -v grep
