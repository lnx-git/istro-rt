#!/bin/bash
CMD="/home/pi/projects/robotour2016/istro_rt2016/istro_rt2016"
echo CONTROL BOARD:
CB_OUT=`for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && continue
        echo "/dev/$devname - $ID_SERIAL"
    )
done |grep ttyUSB |grep FTDI_FT232R_USB_UART_A600e8et`
if [ -z "$CB_OUT" ]; then
	echo "Control board not found."
else
	echo $CB_OUT
	CMD=$CMD" -cb "`echo $CB_OUT|awk '{print $1}'`
fi
 
echo LIDAR:
LIDAR_OUT=`for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && continue
        echo "/dev/$devname - $ID_SERIAL"
    )
done |grep ttyUSB |grep Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001`
if [ -z "$LIDAR_OUT" ]; then
        echo "Lidar not found."
else
	echo $LIDAR_OUT
	CMD=$CMD" -lidar "`echo $LIDAR_OUT|awk '{print $1}'`
fi

echo GPS:
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
	echo $GPS_OUT
	#CMD=$CMD" -gps "`echo $GPS_OUT|awk '{print $1}'`
fi

echo myAHRS:
AHRS_OUT=`for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && continue
        echo "/dev/$devname - $ID_SERIAL"
    )
done |grep ttyACM |grep STMicroelectronics_STM32_Virtual_COM_Port_000001010000`
if [ -z "$AHRS_OUT" ]; then
        echo "myAHRS not found."
else
	echo $AHRS_OUT
	CMD=$CMD" -ahrs "`echo $AHRS_OUT|awk '{print $1}'`
fi

cd /home/pi/projects/robotour2016/istro_rt2016/
rm /home/pi/projects/robotour2016/istro_rt2016/out/lidar.log
rm /home/pi/projects/robotour2016/istro_rt2016/out/robotour_*jpg

CMD=$CMD" -h 10 -m 31"
echo Executing: $CMD
$CMD > out/istro_rt2016.out
