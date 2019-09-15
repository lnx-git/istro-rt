cd ~/projects/istro_rt2019/
rm ~/projects/istro_rt2019/out/lidar.log
rm ~/projects/istro_rt2019/out/rt2019_*jpg
rm ~/projects/istro_rt2019/out/rt2019_*png
rm ~/projects/istro_rt2019/out/rt2019_*kml
rm ~/projects/istro_rt2019/out/istro_rt2019.*
sudo killall gpspipe
rm ~/projects/istro_rt2019/out/gpspipe.log
gpspipe -r -o out/gpspipe.log &
~/projects/istro_rt2019/istro_rt2019 -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -path *1*2
#~/projects/istro_rt2019/istro_rt2019 -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -path XX > out/istro_rt2019.out
sudo killall gpspipe
