call grep "saveImage(): camera_image=" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log01
call grep "saveImage(): vision_image=" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log02
call grep "saveImage(): lidar_image=" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log03
call grep "saveImage(): wmgrid_image=" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log04
call grep "process_thread(): process_angle" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log05
call grep "gps_writeData(): fix=" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log06
call grep "process_readData(): process_change" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log07
call grep "INFO.*calib_process" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log08
call grep "INFO.*wrongway_check" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log09
call grep "wrongway_process" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log10
call grep "navigation point passed" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log11
call grep "navig::navigation" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log12
call grep "loadarea_process" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log13
call grep "ControlBoard::write(): data=.D" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log14
call grep "navmap::navmap_export_kml" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log15
call grep "gps_thread....msg" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log16
call grep -i "error" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log17
call grep "kml\|gps_thread.*nmap" istro_rt2020.log >istro_rt2020.log18

call grep -i "qrscan"  ..\out\istro_rt2020.log > ..\out\istro_rt2020.log90
call grep -i "qrscan\|process_readData(): process_change"  ..\out\istro_rt2020.log > ..\out\istro_rt2020.log91
call grep "ControlBoard::write(): data" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log92
call grep "ControlBoard::write(): data=.D\|ControlBoard::write(): data=.S2\|R:\|obstacle.forward" ..\out\istro_rt2020.log > ..\out\istro_rt2020.log93