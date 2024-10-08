#!/bin/bash
CMD="/home/istrobotics/projects/istro_rt2020/istro_rt2020"
FS="/home/istrobotics/projects/istro_rt2020/out"
TRESHOLD=7000

free_space_mb()
{
 FS=$1

 if [ -n "$FS" ]; then
    df $FS | awk '{print "%d\n", int( $4/1024 ) }'|tail -1
 else
    df . | awk '{print "%d\n", int( $4/1024 ) }'|tail -1
 fi
}

FSMB=$( free_space_mb $FS )

if [ $FSMB -ge $TRESHOLD ]; then
   echo "Freespace is  $(tput setaf 2)OK$(tput sgr 0): $FSMB [MB]"
else
   echo "Freespace is $(tput setaf 1)CRITICAL$(tput sgr 0): $FSMB [MB]"
   echo "Clean up filesystem $FS .... Treshold value is $TRESHOLD [MB]"
   exit -1
fi

cd /home/istrobotics/projects/istro_rt2020/
rm /home/istrobotics/projects/istro_rt2020/out/lidar.json
rm /home/istrobotics/projects/istro_rt2020/out/camera_depth.json
rm /home/istrobotics/projects/istro_rt2020/out/rt2020_*jpg
rm /home/istrobotics/projects/istro_rt2020/out/rt2020_*png
rm /home/istrobotics/projects/istro_rt2020/out/rt2020_*kml
rm /home/istrobotics/projects/istro_rt2020/out/istro_rt2020.log

sudo killall gpspipe
rm /home/istrobotics/projects/istro_rt2020/out/gpspipe.log
mkdir -p out
gpspipe -r -o out/gpspipe.log &

python /home/istrobotics/projects/istro_rt2020/script/serial_scan.py
CMD=$CMD`python /home/istrobotics/projects/istro_rt2020/script/serial_cmd.py`
CMD=$CMD" -nowait -i DIstrobotics -path S1 -vf 24 -vb -17"
#CMD=$CMD" -nowait -i DIstrobotics -path X1*1*2*3 -vf 14 -vb -17"
#CMD=$CMD" -nowait -i DIstrobotics -path *1*2*3 -vf 12 -vb -15"
#CMD=$CMD" -nowait -i DIstrobotics -path *1*2 -vf 12 -vb -15"
#CMD=$CMD" -nowait -i DIstrobotics -path *1*2 -vf 8 -vb -11"
#CMD=$CMD" -nowait -i DIstrobotics -path *1*2 -vf 10 -vb -13"
#CMD=$CMD" -nowait -i DIstrobotics -path *1*2 -vf 12 -vb -15"
#CMD=$CMD" -nowait -i DIstrobotics -path W5"
#CMD=$CMD" -nowait -i DIstrobotics -path M5 -ca 353.29 -cg 297.13"
#CMD=$CMD" -nowait -i DIstrobotics -path W2 -ca 353.29 -cg 297.13 -velocityFwd 9"
#CMD=$CMD" -nowait -i DIstrobotics -path W2"
#CMD=$CMD" -nowait -i DIstrobotics -path *2M6M1M2M3N7"
#CMD=$CMD" -nowait -i DHELLO -navy 90"
#CMD=$CMD" -nowait -ca 0"
#CMD=$CMD" -nowait -cg 350 -path N2E4E2M5M1"
echo Executing: $CMD
$CMD
sudo killall gpspipe
