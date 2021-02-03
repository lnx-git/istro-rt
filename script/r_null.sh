cd ~/projects/istro_rt2020/
rm ~/projects/istro_rt2020/out/lidar.json
rm ~/projects/istro_rt2020/out/camera_depth.json
rm ~/projects/istro_rt2020/out/rt2020_*jpg
rm ~/projects/istro_rt2020/out/rt2020_*png
rm ~/projects/istro_rt2020/out/rt2020_*kml
rm ~/projects/istro_rt2020/out/istro_rt2020.*
sudo killall gpspipe
rm ~/projects/istro_rt2020/out/gpspipe.log
gpspipe -r -o out/gpspipe.log &
~/projects/istro_rt2020/istro_rt2020 -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -path *1*2
#~/projects/istro_rt2020/istro_rt2020 -nowait -cb /dev/null -cb2 /dev/null -lidar /dev/null -path XX > out/istro_rt2020.out
sudo killall gpspipe
