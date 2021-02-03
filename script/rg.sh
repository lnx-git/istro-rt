#!/bin/bash

BIN=istro_rt2020
DIR=/home/istrobotics/projects/istro_rt2020
CMD=$DIR/$BIN
OUTDIR=$DIR/out
TRESHOLD=6000

free_space_mb()
{
 OUTDIR=$1

 if [ -n "$OUTDIR" ]; then
    df $OUTDIR | awk '{print "%d\n", int( $4/1024 ) }'|tail -1
 else
    df . | awk '{print "%d\n", int( $4/1024 ) }'|tail -1
 fi
}

FSMB=$( free_space_mb $OUTDIR )

if [ $FSMB -ge $TRESHOLD ]; then
   echo "Freespace is  $(tput setaf 2)OK$(tput sgr 0): $FSMB [MB]"
else
   echo "Freespace is $(tput setaf 1)CRITICAL$(tput sgr 0): $FSMB [MB]"
   echo "Clean up filesystem $OUTDIR .... Treshold value is $TRESHOLD [MB]"
   exit -1
fi

cd $DIR
rm $OUTDIR/lidar.json
rm $OUTDIR/camera_depth.json
rm $OUTDIR/rt2020_*jpg
rm $OUTDIR/rt2020_*png
rm $OUTDIR/rt2020_*kml
rm $OUTDIR/istro_rt2020.log

sudo killall gpspipe
rm $OUTDIR/gpspipe.log
mkdir -p $OUTDIR
gpspipe -r -o $OUTDIR/gpspipe.log &

python $DIR/script/serial_scan.py
CMD=$CMD`python $DIR/script/serial_cmd.py`
CMD=$CMD" -nowait -i DIstrobotics -path S1 -vf 24 -vb -17"
#CMD=$CMD" -nowait -i DIstrobotics -path X1*1*2*3 -vf 14 -vb -17"
#CMD=$CMD" -nowait -i DIstrobotics -path *1*2*3 -vf 12 -vb -15"
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
#$CMD
gdb -ex=run --args $CMD
sudo killall gpspipe
