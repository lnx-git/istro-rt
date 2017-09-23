import serial.tools.list_ports
list = serial.tools.list_ports.comports()

PORT = ''
for element in list:
    if element[2].find("VID:PID=067B:2303") >= 0:
        #print 'GPS\t|\t ' + element[0] +'\t|\t'+ element[1] +'\t|\t'+ element[2]
        PORT = element[0]

if PORT == '':
  print '/dev/null'
else:
  print PORT