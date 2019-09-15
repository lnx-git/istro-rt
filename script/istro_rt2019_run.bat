rem istro_rt2019.exe -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -i DHELLO -path N1N2 > out/istro_rt2019.log
istro_rt2019.exe -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -i DHELLO -path T2 -vf 7 -vf2 14 -vf3 21  > out/istro_rt2019.log
rem istro_rt2019.exe -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -i DHELLO -path N2*1M4M1*2M3M5*3 -p1lo 18.111 -p1la 49.111 -p2lo 18.222 -p2la 49.222 -p3lo 18.333 -p3la 49.333 -vf 15 -vb -16 > out/istro_rt2019.log
rem istro_rt2019.exe -nowait -cb /dev/null -lidar /dev/null -ahrs /dev/null -navy 15 > out/istro_rt2019.out
rem istro_rt2019.exe -nowait -cb /dev/null -lidar /dev/null -ahrs /dev/null -cg -45 -ca -45 -path S2S1S3 > out/istro_rt2019.out
