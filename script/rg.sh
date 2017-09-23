#!/bin/bash
CMD="/home/odroid/projects/istro_rt2017/istro_rt2017"

cd /home/odroid/projects/istro_rt2017/
rm /home/odroid/projects/istro_rt2017/out/lidar.log
rm /home/odroid/projects/istro_rt2017/out/rt2017_*jpg
rm /home/odroid/projects/istro_rt2017/out/rt2017_*png
rm /home/odroid/projects/istro_rt2017/out/istro_rt2017.log

sudo killall gpspipe
rm /home/odroid/projects/istro_rt2017/out/gpspipe.log
mkdir -p out
gpspipe -r -o out/gpspipe.log &

python /home/odroid/projects/istro_rt2017/script/serial_scan.py
CMD=$CMD`python /home/odroid/projects/istro_rt2017/script/serial_cmd.py`
CMD=$CMD" -nowait -i DIstrobotics  -path M5N7M3"
#CMD=$CMD" -nowait -i DHELLO -navy 90"
#CMD=$CMD" -nowait -ca 0"
#CMD=$CMD" -nowait -cg 350 -path N2E4E2M5M1"
echo Executing: $CMD
gdb -ex=run --args $CMD
sudo killall gpspipe
