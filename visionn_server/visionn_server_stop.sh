#!/bin/bash
sudo pkill -f visionn_server.py
ps -ef|grep visionn_server.py|grep -v grep
