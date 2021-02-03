#!/bin/bash
cd /home/istrobotics/projects/istro_rt2020/script
g++ -g -std=c++11 -Wall logp.cpp -o logp
./logp >> ../logout/logp.log 2>&1 &
ps -ef|grep logp|grep -v grep

