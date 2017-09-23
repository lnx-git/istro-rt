cd ~/projects/istro_rt2017/
rm ~/projects/istro_rt2017/out/lidar.log
rm ~/projects/istro_rt2017/out/rt2017_*jpg
rm ~/projects/istro_rt2017/out/rt2017_*png
rm ~/projects/istro_rt2017/out/istro_rt2017.*
sudo killall gpspipe
rm ~/projects/istro_rt2017/out/gpspipe.log
gpspipe -r -o out/gpspipe.log &
~/projects/istro_rt2017/istro_rt2017 -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -navy 15
#~/projects/istro_rt2017/istro_rt2017 -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -path XX > out/istro_rt2017.out
sudo killall gpspipe
