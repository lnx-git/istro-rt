# complete start-up sequence 

#
chmod +x gpsd_start.sh 
chmod +x logp_start.sh  
chmod +x r.sh 
cp r.sh ..

# check all peripherals (except camera)
python serial_scan.py 

# run gpsd - will be running on background 
python gpsd_init.py
./gpsd_start.sh 

# run log parser - will be running on background 
./logp_start.sh  

# run GPS2 logging - needs to be executed in a new session
python serial_logger.py

# run main application through VNC - graphic window is needed for keyboard input
cd ..
./r.sh