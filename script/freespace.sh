#!/bin/bash

#FS=/media/boot
TRESHOLD=5000

RED='\033[0;31m'
NC='\033[0m' # No Color

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

echo r.sh


