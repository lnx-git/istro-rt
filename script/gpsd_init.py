'''
gpsd_init version 01
'''
from datetime import datetime
import time
import serial
import serial.tools.list_ports

list = serial.tools.list_ports.comports()
gpsport = ''
#gpsport = '/dev/ttyUSB0'
gpsbaudrate = 4800

for element in list:
    if element[2].find("VID:PID=067B:2303") >= 0 and element[1] == "USB-Serial Controller D":
        #print 'GPS\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
        gpsport = element[0]


if gpsport == '':
    print 'GPS not found.'
    exit()

def ReadGPS(ser, count):
    print 'Reading GPS:'
    for x in range(0, count):
        bytes = ser.readline() #reads in bytes followed by a newline
        if len(bytes) > 0:
            if bytes[0] == '$':
                print bytes[:-1] #print to the console

def WriteGPS(ser, data):
    print 'Writing ' + data + ':'
    ser.flushInput()
    ser.write(data+'\r'+'\n')

#ser = serial.Serial(gpsport, gpsbaudrate, timeout=0)
ser = serial.Serial(gpsport, gpsbaudrate)
ser.flushInput()
ser.flushOutput()
ser.flush()

ReadGPS(ser,5)

print '\nPMTK_API_SET_NMEA_OUTPUT - only RMC sentences...'
WriteGPS(ser, '$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29')
ReadGPS(ser,5)

print '\nPMTK_SET_Nav Speed threshold - disable...'
WriteGPS(ser, '$PMTK386,0*23')
ReadGPS(ser,5)

print '\nplease check if GPS is sending only "$GNRMC,'
print  'use "$PMTK447*35" + Enter to check navigation speed threshold (expected response: "$PMTK527,0.00*00")'
WriteGPS(ser, '$PMTK447*35')
ReadGPS(ser,5)

ser.close()
