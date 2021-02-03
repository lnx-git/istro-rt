#!/bin/bash
grep "saveImage(): camera_image=" ../out/istro_rt2020.log > ../out/istro_rt2020.log01
grep "saveImage(): vision_image=" ../out/istro_rt2020.log > ../out/istro_rt2020.log02
grep "saveImage(): lidar_image=" ../out/istro_rt2020.log > ../out/istro_rt2020.log03
grep "saveImage(): wmgrid_image=" ../out/istro_rt2020.log > ../out/istro_rt2020.log04
grep "process_thread(): process_angle" ../out/istro_rt2020.log > ../out/istro_rt2020.log05
grep "gps_writeData(): fix=" ../out/istro_rt2020.log > ../out/istro_rt2020.log06
grep "process_readData(): process_change" ../out/istro_rt2020.log > ../out/istro_rt2020.log07
grep "INFO.*calib_process" ../out/istro_rt2020.log > ../out/istro_rt2020.log08
grep "INFO.*wrongway_check" ../out/istro_rt2020.log > ../out/istro_rt2020.log09
grep "wrongway_process" ../out/istro_rt2020.log > ../out/istro_rt2020.log10
grep "navigation point passed" ../out/istro_rt2020.log > ../out/istro_rt2020.log11
grep "navig::navigation" ../out/istro_rt2020.log > ../out/istro_rt2020.log12
grep "loadarea_process" ../out/istro_rt2020.log > ../out/istro_rt2020.log13
grep "ControlBoard::write(): data=.D" ../out/istro_rt2020.log > ../out/istro_rt2020.log14
grep "navmap::navmap_export_kml" ../out/istro_rt2020.log > ../out/istro_rt2020.log15
grep "gps_thread....msg" ../out/istro_rt2020.log > ../out/istro_rt2020.log16
grep -i "error" ../out/istro_rt2020.log > ../out/istro_rt2020.log17
grep "kml\|gps_thread.*nmap" ../out/istro_rt2020.log > ../out/istro_rt2020.log18
#
grep -i "qrscan"  ../out/istro_rt2020.log > ../out/istro_rt2020.log90
grep -i "qrscan\|process_readData(): process_change"  ../out/istro_rt2020.log > ../out/istro_rt2020.log91
grep "ControlBoard::write(): data" ../out/istro_rt2020.log > ../out/istro_rt2020.log92
grep "ControlBoard::write(): data=.D\|ControlBoard::write(): data=.S2\|R:\|obstacle.forward" ../out/istro_rt2020.log > ../out/istro_rt2020.log93

