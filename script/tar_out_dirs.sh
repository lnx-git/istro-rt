#!/bin/bash
##########################################################
#                                                        #
#  This will tar all out_* and out-*files in SOURCE_DIR  #
#                                                        #
#  Warning: this will also tar out_*.tar files           # 
#           DO NOT RUN TWICE                             #
#                                                        #
##########################################################

SOURCE_DIR=/home/istrobotics/projects/istro_rt2020/

cd $SOURCE_DIR 

for OUTDIR in out[_,-]*; do
  tar -cvvf $OUTDIR.tar $OUTDIR
done

