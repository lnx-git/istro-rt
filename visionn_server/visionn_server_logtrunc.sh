#!/bin/bash
cd /home/istrobotics/projects/istro_rt2020/visionn_server
truncate -s 0 ../logout/visionn_server.log
date -Iseconds >> ../logout/visionn_server.log
truncate -s 0 ../logout/visionn_server2.log
date -Iseconds >> ../logout/visionn_server2.log
