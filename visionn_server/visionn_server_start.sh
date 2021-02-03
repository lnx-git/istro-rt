#!/bin/bash
cd /home/istrobotics/projects/istro_rt2020/visionn_server
date -Iseconds >> ../logout/visionn_server2.log
python3 ./visionn_server.py >> ../logout/visionn_server2.log 2>&1 &
ps -ef|grep visionn_server.py|grep -v grep
