echo 'PMTK_API_SET_NMEA_OUTPUT - only RMC sentences...'
echo -ne '$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n' > `python gps_port.py`
sleep 1
echo 'PMTK_SET_Nav Speed threshold - disable...'
echo -ne '$PMTK386,0*23\r\n' > `python gps_port.py`
sleep 1
echo 'please check if GPS is sending only "$GNRMC,"' 
echo 'use "$PMTK447*35" + Enter to check navigation speed threshold (expected response: "$PMTK527,0.00*00")'
miniterm.py `python gps_port.py` 4800
