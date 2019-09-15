import serial.tools.list_ports
list = serial.tools.list_ports.comports()
print '-'*80
for element in list:
    print element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
print '-'*80
for element in list:
    if element[2].find("VID:PID=2341:0042") >= 0:
        print 'CB\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
for element in list:
    if element[2].find("VID:PID=1A86:7523") >= 0:
        print 'CB2\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
for element in list:
    if element[2].find("VID:PID=067B:2303") >= 0 and element[1] == "USB-Serial Controller D":
        print 'GPS\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
for element in list:
    if element[2].find("VID:PID=067B:2303") >= 0 and element[1] == "USB-Serial Controller":
        print 'GPS2\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
for element in list:
    if element[2].find("VID:PID=10C4:EA60") >= 0:
        print 'LIDAR\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
print '-'*80

