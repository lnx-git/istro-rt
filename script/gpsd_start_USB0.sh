sudo gpsd -b /dev/ttyUSB0 -F /var/run/gpsd.sock
ps -ef|grep gps|grep -v grep
