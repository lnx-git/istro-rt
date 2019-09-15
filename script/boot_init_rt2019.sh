#!/bin/bash
#

echo "Start boot_init at " `date` > /home/istrobotics/projects/istro_rt2019/boot_init_rt2019.log

# Inicializacia logp
cd /home/istrobotics/projects/istro_rt2019/script
/home/istrobotics/projects/istro_rt2019/script/logp_start.sh >> /home/istrobotics/projects/istro_rt2019/boot_init_rt2019.log

# Inicializacia GPS
python /home/istrobotics/projects/istro_rt2019/script/gpsd_init.py >> /home/istrobotics/projects/istro_rt2019/boot_init_rt2019.log

# Start gpsd 
echo "Starting gpsd at " `date` >> /home/istrobotics/projects/istro_rt2019/boot_init_rt2019.log
cd /home/istrobotics/projects/istro_rt2019/script
/home/istrobotics/projects/istro_rt2019/script/gpsd_start.sh
