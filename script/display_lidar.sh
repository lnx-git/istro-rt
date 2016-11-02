cd ~/projects/robotour2016/istro_rt2016/out/
while true
do
ls -altr *lidar.jpg|tail -1
ls -tr *lidar.jpg|tail -1| xargs imshow
done

