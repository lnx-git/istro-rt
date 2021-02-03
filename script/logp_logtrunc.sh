#!/bin/bash
cd /home/istrobotics/projects/istro_rt2020
truncate -s 0 logout/logp.log
date -Iseconds >> logout/logp.log
