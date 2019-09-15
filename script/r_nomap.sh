#!/bin/bash
CMD="/home/istrobotics/projects/istro_rt2019/istro_rt2019"
FS="/home/istrobotics/projects/istro_rt2019/out"
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

cd /home/istrobotics/projects/istro_rt2019/
rm /home/istrobotics/projects/istro_rt2019/out/lidar.log
rm /home/istrobotics/projects/istro_rt2019/out/rt2019_*jpg
rm /home/istrobotics/projects/istro_rt2019/out/rt2019_*png
rm /home/istrobotics/projects/istro_rt2019/out/rt2019_*kml
rm /home/istrobotics/projects/istro_rt2019/out/istro_rt2019.log

sudo killall gpspipe
rm /home/istrobotics/projects/istro_rt2019/out/gpspipe.log
mkdir -p out
gpspipe -r -o out/gpspipe.log &

python /home/istrobotics/projects/istro_rt2019/script/serial_scan.py
CMD=$CMD`python /home/istrobotics/projects/istro_rt2019/script/serial_cmd.py`
CMD=$CMD" -nowait -i DIstrobotics"
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
