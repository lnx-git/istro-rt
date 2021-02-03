#!/bin/bash

if [ -z $1 ]; then
  echo "usage:  mvout dir_name"
  exit -1
fi

cd /home/istrobotics/projects/istro_rt2020

if [ ! -d "out" ]; then
  echo "error: out directory doesn't exist!"
  exit -1
fi

if [ ! -d "logout" ]; then
  echo "error: logout directory doesn't exist!"
  exit -1
fi

if [ -d $1 ]; then
  echo "error: target directory already exists!"
  exit -1
fi

mkdir $1

if [ ! -d $1 ]; then
  echo "error: unable to create target directory!"
  exit -1
fi

mv out/* $1
mv logout/istro* $1
cp logout/visionn_server* $1
cp logout/logp.log $1

#visionn_server/visionn_server_logtrunc.sh
truncate -s 0 logout/visionn_server.log
date -Iseconds >> logout/visionn_server.log
truncate -s 0 logout/visionn_server2.log
date -Iseconds >> logout/visionn_server2.log

#script/logp_logtrunc.sh
truncate -s 0 logout/logp.log
date -Iseconds >> logout/logp.log

echo "done: files were moved to \"$1\"..."
