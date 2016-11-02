cd ~/projects/robotour2016/istro_rt2016/
rm ~/projects/robotour2016/istro_rt2016/out/lidar.log
rm ~/projects/robotour2016/istro_rt2016/out/robotour_*jpg
~/projects/robotour2016/istro_rt2016/istro_rt2016 -h 5 -m 57 -cb /dev/ttyUSB2  -lidar /dev/ttyUSB1 -ahrs /dev/ttyACM0 > out/istro_rt2016.out 
