sudo killall gpsd
sudo rm /var/run/gpsd.sock
sudo systemctl stop gpsd.socket

GPS_OUT=`for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && continue
        echo "/dev/$devname - $ID_SERIAL"
    )
done |grep ttyUSB |grep Prolific_Technology_Inc._USB-Serial_Controller_D`
if [ -z "$GPS_OUT" ]; then
        echo "GPS not found."
else
        GPS_DEV=`echo $GPS_OUT|awk '{print $1}'`
	echo "Starting gpsd on "$GPS_DEV
	echo -ne '$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n' > $GPS_DEV
        sleep 1
        echo -ne '$PMTK386,0*23\r\n' > $GPS_DEV
        sleep 1
	sudo gpsd -b $GPS_DEV -F /var/run/gpsd.sock
	echo "Check:"
	ps -ef|grep gpsd|grep -v grep
fi
