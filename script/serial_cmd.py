import serial.tools.list_ports
list = serial.tools.list_ports.comports()

CB = ''
CB2 = ''
GPS = ''
LIDAR = ''
for element in list:
    if element[2].find("VID:PID=2341:0042") >= 0:
        #print 'CB\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
        CB = ' -cb ' + element[0]
for element in list:
    if element[2].find("VID:PID=1A86:7523") >= 0:
        #print 'CB2\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
        CB2 = ' -cb2 ' + element[0]
for element in list:
    if element[2].find("VID:PID=067B:2303") >= 0:
        #print 'GPS\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
        GPS = ' -gps '  + element[0]
for element in list:
    if element[2].find("VID:PID=10C4:EA60") >= 0:
        #print 'LIDAR\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
        LIDAR = ' -lidar ' + element[0]

CMD = CB + CB2 + GPS + LIDAR
print CMD
