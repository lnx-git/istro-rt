#!/bin/bash
#

echo "Start boot_init at " `date` > /home/istrobotics/projects/istro_rt2020/boot_init_rt2020.log

# Inicializacia logp
cd /home/istrobotics/projects/istro_rt2020/script
/home/istrobotics/projects/istro_rt2020/script/logp_start.sh >> /home/istrobotics/projects/istro_rt2020/boot_init_rt2020.log

# Inicializacia GPS
python /home/istrobotics/projects/istro_rt2020/script/gpsd_init.py >> /home/istrobotics/projects/istro_rt2020/boot_init_rt2020.log

# Start gpsd 
echo "Starting gpsd at " `date` >> /home/istrobotics/projects/istro_rt2020/boot_init_rt2020.log
cd /home/istrobotics/projects/istro_rt2020/script
/home/istrobotics/projects/istro_rt2020/script/gpsd_start.sh
