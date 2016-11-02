cd ~/projects/istro_rt2016/
rm ~/projects/istro_rt2016/out/lidar.log
rm ~/projects/istro_rt2016/out/robotour_*jpg
rm ~/projects/istro_rt2016/out/istro_rt2016.*
sudo killall gpspipe
rm ~/projects/istro_rt2016/out/gpspipe.log
gpspipe -r -o out/gpspipe.log &
~/projects/istro_rt2016/istro_rt2016 -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -navy 15
#~/projects/istro_rt2016/istro_rt2016 -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -path XX > out/istro_rt2016.out
sudo killall gpspipe
